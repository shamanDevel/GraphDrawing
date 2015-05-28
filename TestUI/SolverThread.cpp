#include "SolverThread.h"
#include <boost/log/trivial.hpp>
#include <MILP_lp_solve.h>

SolverThread::SolverThread(QObject* parent)
	: QThread(parent)
{
}


SolverThread::~SolverThread(void)
{
}

void SolverThread::run()
{
	crossingNumber = 0;
	solvedGraphs.reserve(originalGraphs->size());
	solvedGraphs.clear();

	//solve them
	MILP* lp = new MILP_lp_solve();
	OOCMCrossingMinimization* cm = new OOCMCrossingMinimization(lp);
	cm->setAbortFunction([](){
		if (QThread::currentThread()->isInterruptionRequested())
			return CrossingMinimization::ABORT;
		else
			return CrossingMinimization::CONTINUE;
	});
	for (int i=0; i<originalGraphs->size(); ++i) {
		BOOST_LOG_TRIVIAL(info) << "Solve the " << (i+1) << "th component";
		CrossingMinimization::solve_result_t result = cm->solve(originalGraphs->at(i).first, originalGraphs->at(i).second);
		if (isInterruptionRequested()) {
			BOOST_LOG_TRIVIAL(info) << "Terminated by the user";
			crossingNumber = -1;
			break;
		}
		if (result) {
			BOOST_LOG_TRIVIAL(info) << "The " << (i+1) << "th component is solved, crossing number is " << result->second;
			crossingNumber += result->second;
			solvedGraphs.push_back(result->first);
		} else {
			BOOST_LOG_TRIVIAL(warning) << "Unable to solve the " << (i+1) << "th component";
			crossingNumber = -1;
			break;
		}
	}
	BOOST_LOG_TRIVIAL(info) << "Solving completed, total crossing number: " << crossingNumber;
	delete cm;
	delete lp;
}

void SolverThread::setGraphs(const vector< pair<GraphCopy, unordered_map<edge, int> > >& graphs)
{
	originalGraphs = &graphs;
}

int SolverThread::getCrossingNumber() const
{
	return crossingNumber;
}

const vector<GraphCopy>& SolverThread::getSolvedGraphs() const
{
	return solvedGraphs;
}

void SolverThread::terminateSolving()
{
	BOOST_LOG_TRIVIAL(info) << "Termination requested";
	requestInterruption();
}