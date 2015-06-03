#include "SolverThread.h"
#include <QtCore/QElapsedTimer>
#include <boost/log/trivial.hpp>
#include <MILP_lp_solve.h>
#include <exception>

SolverThread::SolverThread(const Graph& originalGraph, QObject* parent)
	: QThread(parent), originalGraph(&originalGraph), showDebugOutput(false)
{
	mutex = new QMutex();
	condition = new QWaitCondition();
}


SolverThread::~SolverThread(void)
{
	for (int i=0; i<deg12.size(); ++i) {
		if (deg12[i] != NULL) delete deg12[i];
	}
	if (bicon != NULL) delete bicon;
	delete condition;
	delete mutex;
}

vector<const GraphCopy*> SolverThread::getSimplifiedComponents()
{
	vector<const GraphCopy*> components(deg12.size());
	for (int i=0; i<deg12.size(); ++i) {
		components[i] = &deg12[i]->getSimplifiedGraph();
	}
	return components;
}
vector<const GraphCopy*> SolverThread::getSolvedComponents()
{
	vector<const GraphCopy*> components(results.size());
	for (int i=0; i<results.size(); ++i) {
		components[i] = &results[i];
	}
	return components;
}
const GraphCopy* SolverThread::getSolvedGraph()
{
	return &solvedGraph;
}

void SolverThread::goOn()
{
	condition->wakeAll();
}

bool SolverThread::simplifyGraph()
{
	QElapsedTimer timer;
	timer.start();
	BOOST_LOG_TRIVIAL(info) << "Simplify graph";

	//split into components
	bicon = new SimplificationBiconnected(*originalGraph);
	const vector<GraphCopy>& components = bicon->getComponents();
	BOOST_LOG_TRIVIAL(info) << "Graph splitted into " << components.size() << " non-planar components";

	if (isInterruptionRequested()) return false;

	//simplify components
	deg12.resize(components.size());
	for (int i=0; i<deg12.size(); ++i) {
		deg12[i] = new SimplificationDeg12(components[i]);
		const GraphCopy& GC = deg12[i]->getSimplifiedGraph();
		BOOST_LOG_TRIVIAL(info) << (i+1) << "th component simplified from " << components[i].numberOfNodes()
			<< " to " << GC.numberOfNodes() << " nodes and from " << components[i].numberOfEdges() << " to "
			<< GC.numberOfEdges() << " edges";
		if (isInterruptionRequested()) return false;
	}

	simplificationTime = timer.elapsed() / 1000.0;
	BOOST_LOG_TRIVIAL(info) << "Graph simplified in " << simplificationTime << " sec";

	return true;
}
void inline assertHasOriginal(const GraphCopy& GC, const node n) {
	node on = GC.original(n);
	assert (on != NULL);
	node v;
	forall_nodes(v, GC.original()) {
		if (v == on) return;
	}
	assert (false); //Illegal original node
}
void inline assertHasOriginal(const GraphCopy& GC, const edge e) {
	assert (GC.original(e) != NULL);
	assertHasOriginal(GC, e->source());
	assertHasOriginal(GC, e->target());
}
bool SolverThread::solveGraph()
{
	QElapsedTimer timer;
	timer.start();

	crossingNumber = 0;

	MILP* lp = new MILP_lp_solve();
	OOCMCrossingMinimization* cm = new OOCMCrossingMinimization(lp);
	cm->enableKuratowskiDebugEnabled(showDebugOutput);
	cm->enableRealizeDebugOutput(showDebugOutput);
	cm->setAbortFunction([](){
		if (QThread::currentThread()->isInterruptionRequested())
			return CrossingMinimization::ABORT;
		else
			return CrossingMinimization::CONTINUE;
	});
	results.clear();
	for (int i=0; i<deg12.size(); ++i) {
		BOOST_LOG_TRIVIAL(info) << "Solve the " << (i+1) << "th component";

#ifdef _DEBUG
		node n;
		forall_nodes (n, deg12[i]->getSimplifiedGraph())
			assertHasOriginal(deg12[i]->getSimplifiedGraph(), n); 
		edge e;
		forall_edges (e, deg12[i]->getSimplifiedGraph())
			assertHasOriginal(deg12[i]->getSimplifiedGraph(), e);
#endif

		try {
			CrossingMinimization::solve_result_t result = cm->solve(deg12[i]->getSimplifiedGraph(), deg12[i]->getEdgeCosts());
			if (isInterruptionRequested()) {
				BOOST_LOG_TRIVIAL(info) << "Terminated by the user";
				crossingNumber = -1;
				return false;
			}
			if (result) {
				BOOST_LOG_TRIVIAL(info) << "The " << (i+1) << "th component is solved, crossing number is " << result->second;
				crossingNumber += result->second;
				results.push_back(std::move(result->first));
			} else {
				BOOST_LOG_TRIVIAL(warning) << "Unable to solve the " << (i+1) << "th component";
				crossingNumber = -1;
				return false;
			}
		} catch (std::exception& e) {
			BOOST_LOG_TRIVIAL(fatal) << "Exception while solving: " << e.what();
			crossingNumber = -1;
			throw;
			return false;
		}
	}
	delete cm;
	delete lp;
	solvingTime = timer.elapsed() / 1000.0;
	BOOST_LOG_TRIVIAL(info) << "Solving completed, total crossing number: " << crossingNumber << ", calulation time: " << solvingTime << " sec";
	return true;
}
bool SolverThread::combineGraph()
{
	QElapsedTimer timer;
	timer.start();
	BOOST_LOG_TRIVIAL(info) << "Reverse simplification";

	vector<GraphCopy> components;
	for (int i=0; i<deg12.size(); ++i) {
		components.push_back(std::move(deg12[i]->reverseSimplification(results[i])));
	}
	solvedGraph = std::move(bicon->reverseSimplification(components));

	combiningTime = timer.elapsed() / 1000.0;
	BOOST_LOG_TRIVIAL(info) << "Graph combined in " << combiningTime << " sec";
	return true;
}

void SolverThread::run()
{
	crossingNumber = -1;

	if (!simplifyGraph()) return;
	emit simplifyingFinished();

	if (isInterruptionRequested()) return;
	mutex->lock();
	condition->wait(mutex);
	mutex->unlock();
	if (isInterruptionRequested()) return;

	if (!solveGraph()) return;
	emit solvingFinished();

	//if (isInterruptionRequested()) return;
	//mutex->lock();
	//condition->wait(mutex);
	//mutex->unlock();
	//if (isInterruptionRequested()) return;

	if (!combineGraph()) return;
	emit combiningFinished();
}


void SolverThread::terminateSolving()
{
	BOOST_LOG_TRIVIAL(info) << "Termination requested";
	requestInterruption();
}

