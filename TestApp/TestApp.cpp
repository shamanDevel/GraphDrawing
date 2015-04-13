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

void TestRealize2() {
	GraphGenerator gen;
	CrossingMinimization cm;
	for (int n=5; n<20; ++n) {
		for (int i=0; i<5; ++i) {
			Graph g = *gen.createRandomGraph(n, max(n*(n-1)/4, n-1));
			int m = num_edges(g);

			//create variables
			vector<CrossingMinimization::crossingInfo> variableInfos 
				= cm.createVariables(g);
			vector<bool> variables(variableInfos.size());
			for (int j=0; j<variables.size(); ++j)
				variables[j] = false;

			//set some variables to true
			int crossings = 0;
			set<CrossingMinimization::edge> deletedEdges;
			for (int j=0; j<variables.size(); ++j) {
				CrossingMinimization::edge e = variableInfos[j].first;
				CrossingMinimization::edge f = variableInfos[j].second;
				if (deletedEdges.count(e) > 0 || deletedEdges.count(f) > 0) {
					continue; //preserve single crossings
				}
				if (RandomInt(0, 5) == 0) {
					variables[j] = true;
					crossings++;
					deletedEdges.insert(e);
					deletedEdges.insert(f);
				}
			}

			//realize graph
			Graph g2 = cm.realize(g, variableInfos, variables);
			int n2 = num_vertices(g2);
			int m2 = num_edges(g2);

			int expectedN = n + crossings;
			int expectedM = m + crossings*2;

			//Test -> Graph should be equivalent
			cout << "expected node count: " << expectedN << ", actual: " << n2 << endl;
			cout << "expected edge count: " << expectedM << ", actual: " << m2 << endl;
		}
	}
}

void TestLinearProgramming()
{
	MILP* lp = new MILP_lp_solve();
	lp->initialize(2);
	lp->setVariableType(1, MILP::VariableType::Integer);
	lp->setVariableType(2, MILP::VariableType::Integer);
			
	int *colno = NULL;
	MILP::real* row = NULL;
	colno = (int *) malloc(2 * sizeof(*colno));
	row = (REAL *) malloc(2 * sizeof(*row));
	colno[0] = 1;
	colno[1] = 2;

	//set objective function
	row[0] = 143;
	row[1] = 60;
	lp->setObjectiveFunction(2, row, colno, MILP::Direction::Maximize);

	lp->setAddConstraintMode(true);
	//first row
	row[0] = 120;
	row[1] = 210;
	lp->addConstraint(2, row, colno, MILP::ConstraintType::LessThanEqual, 15000);

	//second row
	row[0] = 110;
	row[1] = 30;
	lp->addConstraint(2, row, colno, MILP::ConstraintType::LessThanEqual, 4000);
	//third row
	row[0] = 1;
	row[1] = 1;
	lp->addConstraint(2, row, colno, MILP::ConstraintType::LessThanEqual, 75);
	lp->setAddConstraintMode(false);

	//solve
	MILP::real objective;
	MILP::real* variables;
	MILP::SolveResult result = lp->solve(&objective, &variables);
	MILP::SolveResult expected = MILP::SolveResult::Optimal;

	//test
	MILP::real expectedObjective = 6266;
	MILP::real* expectedVariables = new MILP::real[2];
	expectedVariables[0] = 22;
	expectedVariables[1] = 52;
	//cout << "objective: " << objective << " expected: " << expectedObjective;

	lp->printDebug();

	delete colno;
	delete row;
	delete lp;
	delete expectedVariables;
}

void TestMinimization() {
	GraphGenerator gen;
	CrossingMinimization cm;
	for (int n=5; n<20; ++n) {
		//for (int i=0; i<5; ++i) {
			cout << "n=" << n << endl;
			Graph g = *gen.createRandomGraph(n, max(n*(n-1)/2, n-1));
			int m = num_edges(g);

			MILP* milp = new MILP_lp_solve();
			boost::optional< pair<Graph, unsigned int> > result = cm.solve(g, milp);
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

int _tmain(int argc, _TCHAR* argv[])
{
	//TestRealize2();
	//TestLinearProgramming();
	TestMinimization();
	cin.get();
	return 0;
}

