// TestApp.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.
//

#include "stdafx.h"

#include <cstdlib>
#include <sstream>
#include <iostream>
#include <set>
#include <math.h>
#include <string>
#include <cstdio>
#include <GraphGenerator.h>
#include <GraphConverter.h>
#include <CrossingMinimization.h>
#include <OOCMCrossingMinimization.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/boyer_myrvold_planar_test.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/null.hpp>
#include <boost/filesystem.hpp>
#include <boost/graph/graphml.hpp>
#include <ogdf_include.h>
#include <ogdf\planarlayout\PlanarDrawLayout.h>
#include <ogdf\energybased\FMMMLayout.h>
#include <ogdf\energybased\StressMajorizationSimple.h>
#include <ogdf\planarlayout\MixedModelLayout.h>
#include <MILP.h>
#include <MILP_lp_solve.h>

using namespace shaman;
using namespace std;
using namespace ogdf;

#define SAVE_GRAPHS

int RandomInt(int min, int max) 
{
	int r = rand();
	r = r % (max-min+1);
	r += min;
	return r;
}

void SaveGraph(const Graph& G, const char* prefix)
{
#ifdef SAVE_GRAPHS

	int numNodes = G.numberOfNodes();
	int numEdges = G.numberOfEdges();
	//converter
	ogdf::GraphAttributes GA(G, 
		ogdf::GraphAttributes::nodeGraphics | ogdf::GraphAttributes::edgeGraphics
		| ogdf::GraphAttributes::nodeLabel | ogdf::GraphAttributes::edgeStyle
		| ogdf::GraphAttributes::nodeColor );
	SList<node> nodes;
	G.allNodes(nodes);
	for (node n : nodes) {
		stringstream s;
		s << n->index();
		GA.labelNode(n) = s.str().c_str();
		GA.width(n) = GA.height(n) = 10;
	}

	//layout
	BoyerMyrvold bm;
	bool planar = bm.isPlanar(G);
	if (planar) {
#if 1
		//use planar layout
		MixedModelLayout l;
		l.call(GA);
#else
		//use spring layout
		ogdf::FMMMLayout fmmm;
		fmmm.useHighLevelOptions(true);
		fmmm.unitEdgeLength(25.0); 
		fmmm.newInitialPlacement(true);
		fmmm.qualityVersusSpeed(ogdf::FMMMLayout::qvsGorgeousAndEfficient);
		fmmm.call(GA);
#endif
	} else {
#if 1
		//use spring layout
		ogdf::FMMMLayout fmmm;
		fmmm.useHighLevelOptions(true);
		fmmm.unitEdgeLength(10.0 * sqrt(numEdges)); 
		fmmm.newInitialPlacement(true);
		fmmm.qualityVersusSpeed(ogdf::FMMMLayout::qvsGorgeousAndEfficient);
		fmmm.call(GA);
#else
		ogdf::StressMajorization sm;
		sm.call(GA);
#endif
	}

	//save
	stringstream s;
	s << "C:\\Users\\Sebastian\\Documents\\C++\\GraphDrawing\\graphs\\";
	s << prefix;
	s << "_n";
	s << numNodes;
	s << "_e";
	s << numEdges;
	if (planar) {
		s << "_planar";
	}
	//s << ".svg";
	//GA.writeSVG(s.str().c_str());
	s << ".gml";
	GA.writeGML(s.str().c_str());
#endif
}

void TestMinimization() {
	GraphGenerator gen;
	MILP* milp = new MILP_lp_solve();
	CrossingMinimization* cm = new OOCMCrossingMinimization(milp);
	for (int n=5; n<20; ++n) {
		//for (int i=0; i<5; ++i) {
			cout << "n=" << n << endl;
			Graph g = *gen.createRandomGraph(n, max(n*(n-1)/2, n-1));
			int m = g.numberOfEdges();

			boost::optional< pair<Graph, unsigned int> > result = cm->solve(g);
			if (result) {
				cout << "graph solved, count of crossings: " << result->second << endl;
				stringstream s;
				s << "K" << n;
				SaveGraph(result->first, s.str().c_str());
			} else {
				cout << "unable to solve graph" << endl;
			}
			cin.get();
			cout << endl;
		//}
	}
}

struct RomeGraphDescr
{
	string fileName;
	int n;
};

void scanRomeGraphs(string folder, vector<RomeGraphDescr>& target)
{
	//TODO
}

void createRomeGraphsInfo(string folder)
{
	using namespace boost::filesystem;
	path p (folder);   // p reads clearer than argv[1] in the following code
	FILE* file;
	fopen_s(&file, (folder + "Graphs.dat").c_str(), "w");
	try
	{
		assert (exists(p));
		assert (is_directory(p));
		for (directory_iterator it = directory_iterator(p); it != directory_iterator(); ++it)
		{
			path f = it->path();
			if (f.extension() == ".graphml") {
				//load graph
				typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> BoostGraph;
				BoostGraph G;
				ifstream is (f.c_str());
				boost::dynamic_properties properties;
				boost::read_graphml(is, G, properties);
				cout << "Graph " << f.filename() << " loaded";
				int n = num_vertices(G);
				int m = num_edges(G);
				bool planar = boost::boyer_myrvold_planarity_test(G);
				cout << " nodes=" << n << " edges=" << m << " planar=" << planar << endl;
				if (planar) {
					fprintf(file, "%d %d %s\n", n, m, f.filename().string().c_str());
				}
			}
		}
	}
	catch (const filesystem_error& ex)
	{
		cerr << ex.what() << '\n';
	}
	catch (const boost::graph_exception& ex)
	{
		cerr << ex.what() << '\n';
	}
	fclose(file);
}

void OOCM_TestRealizeK8()
{
	GraphGenerator gen;
	BoyerMyrvold bm;
	OOCMCrossingMinimization cm(NULL);
	vector<OOCMCrossingMinimization::crossing> crossings;
	vector<OOCMCrossingMinimization::crossingOrder> crossingOrders;
	OOCMCrossingMinimization::crossingOrderMap_t crossingOrdersMap;
	vector<bool> assignment;

	//Make the K8 planar
	static const int K8crossings[18][4] = {
		{0,2,1,3},
		{0,4,1,7},
		{0,5,1,4},
		{0,5,1,7},
		{0,5,2,4},
		{0,6,2,7},
		{0,6,3,4},
		{0,6,3,7},
		{0,7,3,4},
		{1,5,2,4},
		{1,6,2,4},
		{1,6,2,5},
		{1,6,3,5},
		{1,7,3,4},
		{2,6,3,5},
		{2,7,3,5},
		{2,7,3,6},
		{4,6,5,7}
	};
	static const int K8crossingOrders[24][6] = {
		{0,5,1,7,1,4},
		{0,5,1,7,2,4},
		{0,5,1,4,2,4},
		{0,6,3,4,3,7},
		{0,6,3,4,2,7},
		{0,6,3,7,2,7},
		{1,6,2,4,2,5},
		{1,6,2,4,3,5},
		{1,6,2,5,3,5},
		{1,7,0,5,0,4},
		{1,7,0,5,3,4},
		{1,7,0,4,3,4},
		{2,4,1,6,1,5},
		{2,4,1,6,0,5},
		{2,4,1,5,0,5},
		{2,7,3,5,3,6},
		{2,7,3,5,0,6},
		{2,7,3,6,0,6},
		{3,4,0,6,0,7},
		{3,4,0,6,1,7},
		{3,4,0,7,1,7},
		{3,5,2,7,2,6},
		{3,5,2,7,1,6},
		{3,5,2,6,1,6}
	};
	Graph K8 = *gen.createRandomGraph(8, 8*(8-1)/2);
	assert(!bm.isPlanar(K8));
	cm.createVariables(K8, crossings, crossingOrders);
	assignment.resize(crossings.size() + crossingOrders.size());
	fill (assignment.begin(), assignment.end(), false);
	crossingOrdersMap.clear();
	cm.createCrossingOrdersMap(crossingOrders, crossingOrdersMap);
	unordered_map<node, int> crossingNodes;
	GraphCopy g (K8);
	cm.realize(K8, g, crossings, crossingOrdersMap, assignment, crossingNodes);
	assert(!bm.isPlanar(g));

	//set variables
	int index = 0;
	SList<node> nodeList;
	K8.allNodes(nodeList);
	vector<node> nodes;
	for (const node& n : nodeList) nodes.push_back(n);
#define SEARCH_EDGE(u,v) K8.searchEdge(nodes[(u)], nodes[(v)])
	for (int i=0; i<18; ++i) {
		OOCMCrossingMinimization::crossing c 
			= make_pair(SEARCH_EDGE(K8crossings[i][0], K8crossings[i][1]),
					    SEARCH_EDGE(K8crossings[i][2], K8crossings[i][3]));
		for (; crossings[index] != c; ++index);
		assert(crossings[index] == c);
		assignment[index] = true;
		index++;
	}
	for (int i=0; i<24; ++i) {
		OOCMCrossingMinimization::crossingOrder o
			= make_tuple(SEARCH_EDGE(K8crossingOrders[i][0], K8crossingOrders[i][1]),
					        SEARCH_EDGE(K8crossingOrders[i][2], K8crossingOrders[i][3]),
							SEARCH_EDGE(K8crossingOrders[i][4], K8crossingOrders[i][5]));
		index = 0;
		for (; crossingOrders[index] != o; ++index);
		assert(crossingOrders[index] == o);
		assignment[index + crossings.size()] = true;
		index++;
	}
#undef SEARCH_EDGE
	cout << "K8: " << endl;
	crossingOrdersMap.clear();
	cm.createCrossingOrdersMap(crossingOrders, crossingOrdersMap);
	crossingNodes.clear();
	g = GraphCopy (K8);
	cm.realize(K8, g, crossings, crossingOrdersMap, assignment, crossingNodes);
	assert(bm.isPlanar(g));
	Graph k8 = g;

	//test if the variable entries in the node are correct
	nodeList.clear();
	g.allNodes(nodeList);
	for(const node& n : nodeList) {
		if (crossingNodes.count(n) > 0) {
			int variable = crossingNodes.at(n);
			assert(assignment[variable]);
		}
	}

	//Test if this assignment is also feasible in the lp-model
	MILP* lp = new MILP_lp_solve();
	assert(lp->initialize(crossings.size() + crossingOrders.size()));
	cout << "LP initialized." << endl;
	assert(cm.setObjectiveFunction(crossings, lp));
	cout << "Objective function set." << endl;
	SList<edge> edgeVector;
	K8.allEdges(edgeVector);
	assert(cm.addLinearOrderingConstraints(edgeVector, crossings, crossingOrdersMap, lp));
	cout << "Linear Ordering Constraints added." << endl;
	vector<MILP::real> row(1);
	vector<int> colno(1);
	row[0] = 1;
	lp->setAddConstraintMode(true);
	for (int i=0; i<assignment.size(); ++i) {
		colno[0] = i+1;
		assert(lp->addConstraint(1, &row[0], &colno[0], MILP::ConstraintType::Equal, assignment[i] ? 1 : 0));
	}
	lp->setAddConstraintMode(false);
	cout << "Solution constraints added." << endl;
	MILP::real objective;
	MILP::real* variables;
	MILP::SolveResult solveResult = lp->solve(&objective, &variables);
	cout << "Solved." << endl;
	assert(MILP::SolveResult::Optimal == solveResult);
	assert(18.0 == objective);
}

void K6Bug() 
{
	GraphGenerator gen;
	BoyerMyrvold bm;
	OOCMCrossingMinimization cm(NULL);
	vector<OOCMCrossingMinimization::crossing> crossings;
	vector<OOCMCrossingMinimization::crossingOrder> crossingOrders;
	OOCMCrossingMinimization::crossingOrderMap_t crossingOrdersMap;
	vector<bool> assignment;

	//Create the K6
	Graph K6 = *gen.createRandomGraph(6, 6*5/2);
	SaveGraph(K6, "K6");

	//create variables
	cm.createVariables(K6, crossings, crossingOrders);
	assignment.resize(crossings.size() + crossingOrders.size());
	fill (assignment.begin(), assignment.end(), false);
	crossingOrdersMap.clear();
	cm.createCrossingOrdersMap(crossingOrders, crossingOrdersMap);

	//create LP model
	MILP* lp = new MILP_lp_solve();
	lp->initialize(crossings.size() + crossingOrders.size());
	cm.setObjectiveFunction(crossings, lp);
	SList<edge> edgeVector;
	K6.allEdges(edgeVector);
	cm.addLinearOrderingConstraints(edgeVector, crossings, crossingOrdersMap, lp);

	//set these variables to one: (0,1)x(3,4) (0,1)x(4,5) (0,2)x(4,5) (0,1),(3,4),(4,5) (4,5),(0,1),(0,2)
	for (int i=0; i<crossings.size(); ++i) {
		OOCMCrossingMinimization::crossing c = crossings[i];
		int u1 = c.first->source()->index();
		int v1 = c.first->target()->index();
		int u2 = c.second->source()->index();
		int v2 = c.second->target()->index();
		if ( (u1==0 && v1==1 && u2==3 && v2==4)
			|| (u1==0 && v1==1 && u2==4 && v2==5)
			|| (u1==0 && v1==2 && u2==4 && v2==5) ) {
				assignment[i] = true;
		}
	}
	for (int i=0; i<crossingOrders.size(); ++i) {
		OOCMCrossingMinimization::crossingOrder o = crossingOrders[i];
		int u1 = get<0>(o)->source()->index();
		int v1 = get<0>(o)->target()->index();
		int u2 = get<1>(o)->source()->index();
		int v2 = get<1>(o)->target()->index();
		int u3 = get<2>(o)->source()->index();
		int v3 = get<2>(o)->target()->index();
		if ( (u1==0 && v1==1 && u2==3 && v2==4 && u3==4 && v3==5)
			|| (u1==4 && v1==5 && u2==0 && v2==1 && u3==0 && v3==2) ) {
				assignment[crossings.size() + i] = true;
		}
	}

	//realize graph
	GraphCopy g (K6);
	unordered_map<node, int> crossingNodes;
	cm.realize(K6, g, crossings, crossingOrdersMap, assignment, crossingNodes);
	SaveGraph(g, "K6HalfPlanarized");

	//extract kuratowski subdivision
	SList<KuratowskiWrapper> kuratowski_edges;
	if (bm.planarEmbed(GraphCopySimple(g), kuratowski_edges, BoyerMyrvoldPlanar::doFindUnlimited, true)) 
	{
		//graph is planar
		assert(false);
	}
	else
	{
		cout << "Count of kuratowski subdivisions: " << kuratowski_edges.size() << endl;
		for (KuratowskiWrapper w : kuratowski_edges) {
			cout << "Kuratowki-Subgraph:";
			for (const auto& e : w.edgeList) {
				cout << " (" << e->source()->index() << "," << e->target()->index() << ")" ;
			}
			cout << "  count=" << w.edgeList.size();
			cout << endl;

			if (!cm.addKuratowkiConstraints(edgeVector, crossings, crossingOrdersMap, assignment, w, g, crossingNodes, lp))
				assert (false);

			break; //only one kuratowski subgraph
		}
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	TestMinimization();
	//createRomeGraphsInfo("C:\\Users\\Sebastian\\Documents\\C++\\GraphDrawing\\example-data\\");
	//OOCM_TestRealizeK8();
	//K6Bug();
	cin.get();
	return 0;
}

