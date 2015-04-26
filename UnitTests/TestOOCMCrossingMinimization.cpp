#include "stdafx.h"
#include "CppUnitTest.h"
#include "CppUnitTestAssert.h"

#include <cstdlib>
#include <sstream>
#include <iostream>
#include <set>
#include <math.h>
#include <algorithm>
#include <tuple>
#include <GraphGenerator.h>
#include <GraphConverter.h>
#include <CrossingMinimization.h>
#include <OOCMCrossingMinimization.h>
#include <ogdf_include.h>
#include <ogdf\planarlayout\MixedModelLayout.h>
#include <ogdf\energybased\FMMMLayout.h>
#include <ogdf\energybased\StressMajorizationSimple.h>
#include <ogdf\planarity\BoyerMyrvold.h>
#include <MILP.h>
#include <MILP_lp_solve.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace shaman;
using namespace std;
using namespace ogdf;

#define SAVE_GRAPHS

namespace UnitTests
{		
	#define PRINT_CROSSING(os, c) (os) << " (" << (c).first->source()->index() << "," << (c).first->target()->index() << ")x(" << (c).second->source()->index() << "," << (c).second->target()->index() << ")"

	TEST_CLASS(TestOOCMCrossingMinimization)
	{
	public:

		TEST_METHOD(OOCM_TestCreateVariables)
		{
			GraphGenerator gen;
			OOCMCrossingMinimization cm(NULL);
			for (int n=5; n<10; ++n) {
				for (int i=0; i<2; ++i) {
					//create graph
					int m = i==0 ? n*(n-1)/2 : RandomInt(n-1, n*(n-1)/2);
					Graph g = *gen.createRandomGraph(n, m);

					//create variables
					vector<OOCMCrossingMinimization::crossing> outCrossings;
					vector<OOCMCrossingMinimization::crossingOrder> outCrossingOrders;
					cm.createVariables(g, outCrossings, outCrossingOrders);

					stringstream s;
					s << "n=" << n << " m=" << m << " -> variables=" 
						<< (outCrossings.size() + outCrossingOrders.size()) << endl;
					//for (OOCMCrossingMinimization::crossing i : outCrossings) {
					//	s << "  {(" << i.first.first << "," << i.first.second;
					//	s << ")x(" << i.second.first << "," << i.second.second;
					//	s << ")}";
					//}
					//s << endl;
					//for (OOCMCrossingMinimization::crossingOrder o : outCrossingOrders) {
					//	s << " ((" << get<0>(o).first << "," << get<0>(o).second
					//		<< ");(" << get<1>(o).first << "," << get<1>(o).second
					//		<< ");(" << get<2>(o).first << "," << get<2>(o).second
					//		<< "))";
					//}
					s << endl;
					Logger::WriteMessage(s.str().c_str());
				}
			}
		}

		TEST_METHOD(OOCM_TestRealize1) 
		{
			GraphGenerator gen;
			OOCMCrossingMinimization cm(NULL);
			for (int n=5; n<10; ++n) {
				for (int i=0; i<2; ++i) {
					//create graph
					int m = i==0 ? n*(n-1)/2 : RandomInt(n-1, n*(n-1)/2);
					Graph g = *gen.createRandomGraph(n, m);

					//create variables
					vector<OOCMCrossingMinimization::crossing> crossings;
					vector<OOCMCrossingMinimization::crossingOrder> crossingOrders;
					cm.createVariables(g, crossings, crossingOrders);

					//set assignment to false
					vector<bool> assignment (crossings.size() + crossingOrders.size());
					fill (assignment.begin(), assignment.end(), false);

					//realize graph
					stringstream s;
					OOCMCrossingMinimization::crossingOrderMap_t crossingOrdersMap;
					cm.createCrossingOrdersMap(crossingOrders, crossingOrdersMap);
					unordered_map<node, int> crossingNodes;
					GraphCopy g2 (g);
					cm.realize(g, g2, crossings, crossingOrdersMap, assignment, crossingNodes, s);
					int n2 = g2.numberOfNodes();
					int m2 = g2.numberOfEdges();

					//Test -> Graph should be equivalent
					Assert::AreEqual(n, n2, L"Node count does not match", LINE_INFO());
					Assert::AreEqual(m, m2, L"Edge count does not match", LINE_INFO());
				}
			}
		}

		TEST_METHOD(OOCM_TestRealizeK6) 
		{
			GraphGenerator gen;
			BoyerMyrvold bm;
			OOCMCrossingMinimization cm(NULL);
			vector<OOCMCrossingMinimization::crossing> crossings;
			vector<OOCMCrossingMinimization::crossingOrder> crossingOrders;
			OOCMCrossingMinimization::crossingOrderMap_t crossingOrdersMap;
			vector<bool> assignment;
			stringstream rs;

			//Make the K6 planar
			Graph K6 = *gen.createRandomGraph(6, 6*(6-1)/2);
			Assert::IsFalse(bm.isPlanar(K6), L"K6 should not be planar", LINE_INFO());
			cm.createVariables(K6, crossings, crossingOrders);
			assignment.resize(crossings.size() + crossingOrders.size());
			fill (assignment.begin(), assignment.end(), false);
			rs = stringstream();
			crossingOrdersMap.clear();
			cm.createCrossingOrdersMap(crossingOrders, crossingOrdersMap);
			unordered_map<node, int> crossingNodes;
			GraphCopy g (K6);
			cm.realize(K6, g, crossings, crossingOrdersMap, assignment, crossingNodes, rs);
			Assert::IsFalse(bm.isPlanar(g), L"realized K6 should not be planar", LINE_INFO());
			//specify crossings between (0,1)x(3,5); (0,4)x(2,3); (1,2)x(4,5)
			int countOfOnes = 0;
			SList<node> nodeList;
			g.allNodes(nodeList);
			vector<node> nodes(nodeList.size());
			for (const node& n : nodeList) nodes[n->index()] = n;
			stringstream str;
			str << "crossings: ";
			for (auto c : crossings) {
				PRINT_CROSSING(str, c);
			}
			str << endl;
			Logger::WriteMessage(str.str().c_str());
#define SEARCH_EDGE(g,u,v) (g).searchEdge(nodes[(u)], nodes[(v)])
			int v1 = crossings[0].first->source()->index();
			int u1 = crossings[0].first->target()->index();
			int v2 = crossings[0].second->source()->index();
			int u2 = crossings[0].second->target()->index();
			str = stringstream();
			OOCMCrossingMinimization::crossing c = make_pair( SEARCH_EDGE(g, v1,u1), SEARCH_EDGE(g, v2,u2));
			str << "c1: ";
			PRINT_CROSSING(str, crossings[0]);
			str << "  c2: ";
			PRINT_CROSSING(str, c);
			str << endl;
			Logger::WriteMessage(str.str().c_str());
			Assert::IsTrue(crossings[0] == c, 
				L"unable to compare edges", LINE_INFO());
			for (int i=0; i<crossings.size(); ++i) {
				OOCMCrossingMinimization::crossing c = crossings[i];
				if (c == make_pair( SEARCH_EDGE(g, 0,1), SEARCH_EDGE(g, 3,5) )
					|| c == make_pair( SEARCH_EDGE(g, 0,4), SEARCH_EDGE(g, 2,3) )
					|| c == make_pair( SEARCH_EDGE(g, 1,2), SEARCH_EDGE(g, 4,5) )) {
						assignment[i] = true;
						countOfOnes++;
				}
			}
#undef SEARCH_EDGE
			Assert::AreEqual(3, countOfOnes, L"Three crossings should be set", LINE_INFO());
			//realize
			rs = stringstream();
			rs << "K6: " << endl;
			crossingOrdersMap.clear();
			cm.createCrossingOrdersMap(crossingOrders, crossingOrdersMap);
			g = GraphCopy(K6);
			cm.realize(K6, g, crossings, crossingOrdersMap, assignment, crossingNodes, rs);
			Logger::WriteMessage(rs.str().c_str());
			Assert::IsTrue(bm.isPlanar(g), L"realized K6 should now be planar", LINE_INFO());
			Graph k6 = g;

			//test if the variable entries in the node are correct
			g.allNodes(nodeList);
			for(const node& n : nodeList) {
				if (crossingNodes.count(n) > 0) {
					int variable = crossingNodes.at(n);
					Assert::IsTrue(assignment[variable], L"named variable is not correct", LINE_INFO());
				}
			}

			//Test if this assignment is also feasible in the lp-model
			MILP* lp = new MILP_lp_solve();
			Assert::IsTrue(lp->initialize(crossings.size() + crossingOrders.size()), L"unable to initialize LP", LINE_INFO());
			Logger::WriteMessage("LP initialized.");
			Assert::IsTrue(cm.setObjectiveFunction(crossings, lp), L"unable to set objective function", LINE_INFO());
			Logger::WriteMessage("Objective function set.");
			SList<edge> edgeVector;
			K6.allEdges(edgeVector);
			Assert::IsTrue(cm.addLinearOrderingConstraints(edgeVector, crossings, crossingOrdersMap, lp),
				L"unable to add linear ordering constraints", LINE_INFO());
			Logger::WriteMessage("Linear Ordering Constraints added.");
			vector<MILP::real> row(1);
			vector<int> colno(1);
			row[0] = 1;
			lp->setAddConstraintMode(true);
			for (int i=0; i<assignment.size(); ++i) {
				colno[0] = i+1;
				Assert::IsTrue(lp->addConstraint(1, &row[0], &colno[0], MILP::ConstraintType::Equal, assignment[i] ? 1 : 0));
			}
			lp->setAddConstraintMode(false);
			Logger::WriteMessage("Solution constraints added.");
			MILP::real objective;
			MILP::real* variables;
			MILP::SolveResult solveResult = lp->solve(&objective, &variables);
			Logger::WriteMessage("Solved.");
			Assert::AreEqual((int) MILP::SolveResult::Optimal, (int) solveResult, L"solver does not return Optimal solution", LINE_INFO());
			Assert::AreEqual(3.0, objective, L"wrong objective", LINE_INFO());

#ifdef SAVE_GRAPHS
			SaveGraph(k6, "K6");
#endif

		}

		TEST_METHOD(OOCM_TestRealizeK8)
		{
			GraphGenerator gen;
			BoyerMyrvold bm;
			OOCMCrossingMinimization cm(NULL);
			vector<OOCMCrossingMinimization::crossing> crossings;
			vector<OOCMCrossingMinimization::crossingOrder> crossingOrders;
			OOCMCrossingMinimization::crossingOrderMap_t crossingOrdersMap;
			vector<bool> assignment;
			stringstream rs;

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
			Assert::IsFalse(bm.isPlanar(K8), L"K8 should not be planar", LINE_INFO());
			cm.createVariables(K8, crossings, crossingOrders);
			assignment.resize(crossings.size() + crossingOrders.size());
			fill (assignment.begin(), assignment.end(), false);
			rs = stringstream();
			crossingOrdersMap.clear();
			cm.createCrossingOrdersMap(crossingOrders, crossingOrdersMap);
			unordered_map<node, int> crossingNodes;
			GraphCopy g (K8);
			cm.realize(K8, g, crossings, crossingOrdersMap, assignment, crossingNodes, rs);
			Assert::IsFalse(bm.isPlanar(g), L"realized K8 should not be planar", LINE_INFO());

			//set variables
			int index = 0;
			SList<node> nodeList;
			g.allNodes(nodeList);
			vector<node> nodes;
			for (const node& n : nodeList) nodes.push_back(n);
#define SEARCH_EDGE(u,v) g.searchEdge(nodes[(u)], nodes[(v)])
			for (int i=0; i<18; ++i) {
				OOCMCrossingMinimization::crossing c 
					= make_pair(SEARCH_EDGE(K8crossings[i][0], K8crossings[i][1]),
					            SEARCH_EDGE(K8crossings[i][2], K8crossings[i][3]));
				for (; crossings[index] != c; ++index);
				Assert::IsTrue(crossings[index] == c);
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
				Assert::IsTrue(crossingOrders[index] == o);
				assignment[index + crossings.size()] = true;
				index++;
			}
#undef SEARCH_EDGE
			rs = stringstream();
			rs << "K8: " << endl;
			crossingOrdersMap.clear();
			cm.createCrossingOrdersMap(crossingOrders, crossingOrdersMap);
			crossingNodes.clear();
			g = GraphCopy (K8);
			cm.realize(K8, g, crossings, crossingOrdersMap, assignment, crossingNodes, rs);
			Logger::WriteMessage(rs.str().c_str());
			Assert::IsTrue(bm.isPlanar(g), L"realized K8 should now be planar", LINE_INFO());
			Graph k8 = g;

			//test if the variable entries in the node are correct
			nodeList.clear();
			g.allNodes(nodeList);
			for(const node& n : nodeList) {
				if (crossingNodes.count(n) > 0) {
					int variable = crossingNodes.at(n);
					Assert::IsTrue(assignment[variable], L"named variable is not correct", LINE_INFO());
				}
			}

			//Test if this assignment is also feasible in the lp-model
			MILP* lp = new MILP_lp_solve();
			Assert::IsTrue(lp->initialize(crossings.size() + crossingOrders.size()), L"unable to initialize LP", LINE_INFO());
			Logger::WriteMessage("LP initialized.");
			Assert::IsTrue(cm.setObjectiveFunction(crossings, lp), L"unable to set objective function", LINE_INFO());
			Logger::WriteMessage("Objective function set.");
			SList<edge> edgeVector;
			K8.allEdges(edgeVector);
			Assert::IsTrue(cm.addLinearOrderingConstraints(edgeVector, crossings, crossingOrdersMap, lp),
				L"unable to add linear ordering constraints", LINE_INFO());
			Logger::WriteMessage("Linear Ordering Constraints added.");
			vector<MILP::real> row(1);
			vector<int> colno(1);
			row[0] = 1;
			lp->setAddConstraintMode(true);
			for (int i=0; i<assignment.size(); ++i) {
				colno[0] = i+1;
				Assert::IsTrue(lp->addConstraint(1, &row[0], &colno[0], MILP::ConstraintType::Equal, assignment[i] ? 1 : 0));
			}
			lp->setAddConstraintMode(false);
			Logger::WriteMessage("Solution constraints added.");
			MILP::real objective;
			MILP::real* variables;
			MILP::SolveResult solveResult = lp->solve(&objective, &variables);
			Logger::WriteMessage("Solved.");
			Assert::AreEqual((int) MILP::SolveResult::Optimal, (int) solveResult, L"solver does not return Optimal solution", LINE_INFO());
			Assert::AreEqual(18.0, objective, L"wrong objective", LINE_INFO());

#ifdef SAVE_GRAPHS
			SaveGraph(k8, "K8");
#endif
		}

		static int RandomInt(int min, int max) 
		{
			int r = rand();
			r = r % (max-min+1);
			r += min;
			return r;
		}

				static void SaveGraph(const Graph& G, const char* prefix)
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
			}

			//layout
			BoyerMyrvold bm;
			bool planar = bm.isPlanar(G);
			if (planar) {
#if 0
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
			s << ".svg";
			GA.writeSVG(s.str().c_str());
#endif
		}

	};
}