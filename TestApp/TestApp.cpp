// TestApp.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.
//

#include "stdafx.h"
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <set>
#include <math.h>
#include <GraphGenerator.h>
#include <GraphConverter.h>
#include <CrossingMinimization.h>
#include <SOCMCrossingMinimization.h>
#include <OOCMCrossingMinimization.h>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/boyer_myrvold_planar_test.hpp>
#include <ogdf_include.h>
#include <ogdf\planarlayout\PlanarDrawLayout.h>
#include <ogdf\energybased\FMMMLayout.h>
#include <ogdf\energybased\StressMajorizationSimple.h>
#include <MILP.h>
#include <MILP_lp_solve.h>

using namespace shaman;
using namespace std;

int RandomInt(int min, int max) 
{
	int r = rand();
	r = r % (max-min+1);
	r += min;
	return r;
}

void TestMinimization() {
	GraphGenerator gen;
	MILP* milp = new MILP_lp_solve();
	CrossingMinimization* cm = new SOCMCrossingMinimization(milp);
	for (int n=5; n<20; ++n) {
		//for (int i=0; i<5; ++i) {
			cout << "n=" << n << endl;
			Graph g = *gen.createRandomGraph(n, max(n*(n-1)/2, n-1));
			int m = num_edges(g);

			boost::optional< pair<Graph, unsigned int> > result = cm->solve(g);
			if (result) {
				cout << "graph solved, count of crossings: " << result->second << endl;
			} else {
				cout << "unable to solve graph" << endl;
			}
			cin.get();
			cout << endl;
		//}
	}
}

void OOCM_TestRealize2()
{
	GraphGenerator gen;
	OOCMCrossingMinimization cm(NULL);
	vector<OOCMCrossingMinimization::crossing> crossings;
	vector<OOCMCrossingMinimization::crossingOrder> crossingOrders;
	map<OOCMCrossingMinimization::crossingOrder, int> crossingOrdersMap;
	vector<bool> assignment;
	stringstream rs;

	//Make the K6 planar
	Graph K6 = *gen.createRandomGraph(6, 6*(6-1)/2);
	//Assert::IsFalse(boost::boyer_myrvold_planarity_test(K6), L"K6 should not be planar", LINE_INFO());
	cm.createVariables(K6, crossings, crossingOrders);
	assignment.resize(crossings.size() + crossingOrders.size());
	fill (assignment.begin(), assignment.end(), false);
	rs = stringstream();
	crossingOrdersMap.clear();
	cm.createCrossingOrdersMap(crossingOrders, crossingOrdersMap);
	Graph g = cm.realize(K6, crossings, crossingOrdersMap, assignment, rs);
	//Assert::IsFalse(boost::boyer_myrvold_planarity_test(g), L"realized K6 should not be planar", LINE_INFO());
	//specify crossings between (0,1)x(3,5); (0,4)x(2,3); (1,2)x(4,5)
	int countOfOnes = 0;
	for (int i=0; i<crossings.size(); ++i) {
		OOCMCrossingMinimization::crossing c = crossings[i];
		if (c == make_pair( make_pair(0,1), make_pair(3,5) )
			|| c == make_pair( make_pair(0,4), make_pair(2,3) )
			|| c == make_pair( make_pair(1,2), make_pair(4,5) )) {
				assignment[i] = true;
				countOfOnes++;
		}
	}
	//Assert::AreEqual(3, countOfOnes, L"Three crossings should be set", LINE_INFO());
	//realize
	rs = stringstream();
	rs << "K6: " << endl;
	crossingOrdersMap.clear();
	cm.createCrossingOrdersMap(crossingOrders, crossingOrdersMap);
	g = cm.realize(K6, crossings, crossingOrdersMap, assignment, rs);
	cout << rs.str();
	//Logger::WriteMessage(rs.str().c_str());
	//Assert::IsTrue(boost::boyer_myrvold_planarity_test(g), L"realized K6 should now be planar", LINE_INFO());
	Graph k6 = g;

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
	//Assert::IsFalse(boost::boyer_myrvold_planarity_test(K8), L"K8 should not be planar", LINE_INFO());
	cm.createVariables(K8, crossings, crossingOrders);
	assignment.resize(crossings.size() + crossingOrders.size());
	fill (assignment.begin(), assignment.end(), false);
	rs = stringstream();
	crossingOrdersMap.clear();
	cm.createCrossingOrdersMap(crossingOrders, crossingOrdersMap);
	g = cm.realize(K8, crossings, crossingOrdersMap, assignment, rs);
	//Assert::IsFalse(boost::boyer_myrvold_planarity_test(g), L"realized K8 should not be planar", LINE_INFO());

	//set variables
	int index = 0;
	for (int i=0; i<18; ++i) {
		OOCMCrossingMinimization::crossing c 
			= make_pair(make_pair(K8crossings[i][0], K8crossings[i][1]),
					    make_pair(K8crossings[i][2], K8crossings[i][3]));
		for (; crossings[index] != c; ++index);
		//Assert::IsTrue(crossings[index] == c);
		assignment[index] = true;
		index++;
	}
	for (int i=0; i<24; ++i) {
		OOCMCrossingMinimization::crossingOrder o
			= make_tuple(make_pair(K8crossingOrders[i][0], K8crossingOrders[i][1]),
					        make_pair(K8crossingOrders[i][2], K8crossingOrders[i][3]),
							make_pair(K8crossingOrders[i][4], K8crossingOrders[i][5]));
		index = 0;
		for (; crossingOrders[index] != o; ++index);
		//Assert::IsTrue(crossingOrders[index] == o);
		assignment[index + crossings.size()] = true;
		index++;
	}
	rs = stringstream();
	rs << "K8: " << endl;
	crossingOrdersMap.clear();
	cm.createCrossingOrdersMap(crossingOrders, crossingOrdersMap);
	g = cm.realize(K8, crossings, crossingOrdersMap, assignment, rs);
	cout << rs.str();
	//Logger::WriteMessage(rs.str().c_str());
	//Assert::IsTrue(boost::boyer_myrvold_planarity_test(g), L"realized K8 should now be planar", LINE_INFO());
	Graph k8 = g;

#ifdef SAVE_GRAPHS
	TestAndSaveGraph(k6, "K6");
	TestAndSaveGraph(k8, "K8");
#endif
}

int _tmain(int argc, _TCHAR* argv[])
{
	//TestRealize2();
	//TestLinearProgramming();
	//TestMinimization();
	OOCM_TestRealize2();
	cin.get();
	return 0;
}

