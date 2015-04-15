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
#include <boost/graph/connected_components.hpp>
#include <boost/graph/boyer_myrvold_planar_test.hpp>
#include <ogdf_include.h>
#include <ogdf\planarlayout\MixedModelLayout.h>
#include <ogdf\energybased\FMMMLayout.h>
#include <ogdf\energybased\StressMajorizationSimple.h>
#include <MILP.h>
#include <MILP_lp_solve.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace shaman;
using namespace std;

#define SAVE_GRAPHS

namespace UnitTests
{		
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
					Graph g2 = cm.realize(g, crossings, crossingOrders, assignment, s);
					int n2 = num_vertices(g2);
					int m2 = num_edges(g2);

					//Test -> Graph should be equivalent
					Assert::AreEqual(n, n2, L"Node count does not match", LINE_INFO());
					Assert::AreEqual(m, m2, L"Edge count does not match", LINE_INFO());
				}
			}
		}

		TEST_METHOD(OOCM_TestRealize2) 
		{
			GraphGenerator gen;
			OOCMCrossingMinimization cm(NULL);
			vector<OOCMCrossingMinimization::crossing> crossings;
			vector<OOCMCrossingMinimization::crossingOrder> crossingOrders;
			vector<bool> assignment;
			stringstream rs;

			//Make the K6 planar
			Graph K6 = *gen.createRandomGraph(6, 6*(6-1)/2);
			Assert::IsFalse(boost::boyer_myrvold_planarity_test(K6), L"K6 should not be planar", LINE_INFO());
			cm.createVariables(K6, crossings, crossingOrders);
			assignment.resize(crossings.size() + crossingOrders.size());
			fill (assignment.begin(), assignment.end(), false);
			rs = stringstream();
			Graph g = cm.realize(K6, crossings, crossingOrders, assignment, rs);
			Assert::IsFalse(boost::boyer_myrvold_planarity_test(g), L"realized K6 should not be planar", LINE_INFO());
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
			Assert::AreEqual(3, countOfOnes, L"Three crossings should be set", LINE_INFO());
			//realize
			rs = stringstream();
			rs << "K6: " << endl;
			g = cm.realize(K6, crossings, crossingOrders, assignment, rs);
			Logger::WriteMessage(rs.str().c_str());
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
			Assert::IsFalse(boost::boyer_myrvold_planarity_test(K8), L"K8 should not be planar", LINE_INFO());
			cm.createVariables(K8, crossings, crossingOrders);
			assignment.resize(crossings.size() + crossingOrders.size());
			fill (assignment.begin(), assignment.end(), false);
			rs = stringstream();
			g = cm.realize(K8, crossings, crossingOrders, assignment, rs);
			Assert::IsFalse(boost::boyer_myrvold_planarity_test(g), L"realized K8 should not be planar", LINE_INFO());

			//set variables
			int index = 0;
			for (int i=0; i<18; ++i) {
				OOCMCrossingMinimization::crossing c 
					= make_pair(make_pair(K8crossings[i][0], K8crossings[i][1]),
					            make_pair(K8crossings[i][2], K8crossings[i][3]));
				for (; crossings[index] != c; ++index);
				Assert::IsTrue(crossings[index] == c);
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
				Assert::IsTrue(crossingOrders[index] == o);
				assignment[index + crossings.size()] = true;
				index++;
			}
			rs = stringstream();
			rs << "K8: " << endl;
			g = cm.realize(K8, crossings, crossingOrders, assignment, rs);
			Logger::WriteMessage(rs.str().c_str());
			//Assert::IsTrue(boost::boyer_myrvold_planarity_test(g), L"realized K8 should now be planar", LINE_INFO());
			Graph k8 = g;

#ifdef SAVE_GRAPHS
			TestAndSaveGraph(k6, "K6");
			TestAndSaveGraph(k8, "K8");
#endif
		}

		static int RandomInt(int min, int max) 
		{
			int r = rand();
			r = r % (max-min+1);
			r += min;
			return r;
		}

		static void TestAndSaveGraph(const Graph& g, const char* prefix)
		{
			int numNodes = num_vertices(g);
			int numEdges = num_edges(g);
			//convert
			ogdf::Graph G;
			ogdf::GraphAttributes GA(G, 
				ogdf::GraphAttributes::nodeGraphics | ogdf::GraphAttributes::edgeGraphics
				| ogdf::GraphAttributes::nodeLabel | ogdf::GraphAttributes::edgeStyle
				| ogdf::GraphAttributes::nodeColor );
			GraphConverter::Settings settings;
			settings.labelNodes = true;
			settings.nodeSize = 10.0;
			settings.edgeWidth = 1.0;
			settings.nodeColor = "#AAAAAA";
			GraphConverter::convert(g, G, GA, settings);
			int numNodes2 = G.numberOfNodes();
			int numEdges2 = G.numberOfEdges();
			Assert::AreEqual(numNodes, numNodes2, L"Node count is not equal", LINE_INFO());
			Assert::AreEqual(numEdges, numEdges2, L"Edge count is not equal", LINE_INFO());
			//layout
			bool planar = boost::boyer_myrvold_planarity_test(g);
			if (planar) {
#if 1
				//use planar layout
				ogdf::MixedModelLayout l;
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
			cout << "write graph to " << s.str() << endl;
			//Assert::Fail(ToString(s.str()).c_str());
		}

	};
}