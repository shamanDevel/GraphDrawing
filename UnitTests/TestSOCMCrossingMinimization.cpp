#include "stdafx.h"
#include "CppUnitTest.h"
#include "CppUnitTestAssert.h"

#include <cstdlib>
#include <sstream>
#include <iostream>
#include <set>
#include <math.h>
#include <GraphGenerator.h>
#include <GraphConverter.h>
#include <CrossingMinimization.h>
#include <SOCMCrossingMinimization.h>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/boyer_myrvold_planar_test.hpp>
#include <ogdf_include.h>
#include <ogdf\planarlayout\PlanarDrawLayout.h>
#include <ogdf\energybased\FMMMLayout.h>
#include <ogdf\energybased\StressMajorizationSimple.h>
#include <MILP.h>
#include <MILP_lp_solve.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace shaman;
using namespace std;

//#define SAVE_GRAPHS

namespace UnitTests
{		
	TEST_CLASS(TestSOCMCrossingMinimization)
	{
	public:

		TEST_METHOD(TestCrossingNumber)
		{
			for (int n=2; n<20; ++n) {
				for (int i=0; i<5; ++i) {
					int m = RandomInt(3*n-6, n*(n-1)/2);
					int crUpper = CrossingMinimization::crKnUpper(n);
					int crLower = CrossingMinimization::crGLower(n, m);
					Assert::IsTrue(crLower <= crUpper, L"Lower bound is higher than the upper bound", LINE_INFO());
#if 0
					stringstream s;
					s << "n=" << n;
					s << " m=" << m;
					s << " --> ";
					s << crLower << " <= cr(G) <= " << crUpper;
					s << endl;
					Logger::WriteMessage(s.str().c_str());
#endif
				}
			}
		}

		TEST_METHOD(SOCM_TestSplitGraph) {
			GraphGenerator gen;
			SOCMCrossingMinimization cm(NULL);
			for (int n=5; n<20; ++n) {
				Graph g = *gen.createRandomGraph(n, max(n*(n-1)/4, n-1));
				int m = num_edges(g);
				int l = m;
				//split graph
				cm.splitGraph(g);
				int n2 = num_vertices(g);
				int m2 = num_edges(g);
				int nExp = n + (l-1)*m;
				int mExp = l*m;
				Assert::AreEqual(nExp, n2, L"Node count is wrong", LINE_INFO());
				Assert::AreEqual(mExp, m2, L"Edge count is wrong", LINE_INFO());
				vector<int> nodeVec(n2);
				int numComp = boost::connected_components(g, &nodeVec[0]);
				Assert::AreEqual(1, numComp, L"Graph is not connected anymore", LINE_INFO());

#ifdef SAVE_GRAPHS
				TestAndSaveGraph(g, "subdivision");
#endif
			}
		}

		TEST_METHOD(SOCM_TestCreateVariables)
		{
			GraphGenerator gen;
			SOCMCrossingMinimization cm(NULL);
			for (int n=5; n<50; ++n) {
				for (int i=0; i<5; ++i) {
					//create graph
					int m = i==0 ? n*(n-1)/2 : RandomInt(n-1, n*(n-1)/2);
					Graph g = *gen.createRandomGraph(n, m);
					//create variables
					vector<SOCMCrossingMinimization::crossingInfo> variableInfos 
						= cm.createVariables(g);
					for (SOCMCrossingMinimization::crossingInfo i : variableInfos) {
						Assert::IsTrue(i.first.first < i.first.second, L"Edge is orientated the other way", LINE_INFO());
						Assert::IsTrue(i.second.first < i.second.second, L"Edge is orientated the other way", LINE_INFO());
						Assert::IsTrue(i.first.first <= i.second.first, L"Crossing is ordered the other way", LINE_INFO());
						if (i.first.first == i.second.second) {
							Assert::IsTrue(i.first.second < i.second.second, L"Crossing is not ordered correctly", LINE_INFO());
						}
					}
					sort( variableInfos.begin(), variableInfos.end() );
					Assert::IsTrue(adjacent_find( variableInfos.begin(), variableInfos.end() )
						== variableInfos.end(), L"Variables are not unique", LINE_INFO());
		
					stringstream s;
					s << "n=" << n << " m=" << m << " -> variables=" << variableInfos.size() << endl;
					Logger::WriteMessage(s.str().c_str());

					//stringstream s;
					//s << "n=" << n << " m=" << m << " -> variables=" << variableInfos.size() << endl;
					////for (CrossingMinimization::variableInfo i : variableInfos) {
					////	s << "  (" << i.first.first << "," << i.first.second;
					////	s << ") x (" << i.second.first << "," << i.second.second;
					////	s << ")" << endl;
					////}
					//Logger::WriteMessage(s.str().c_str());
				}
			}
		}

		TEST_METHOD(SOCM_TestRealize) {
			GraphGenerator gen;
			SOCMCrossingMinimization cm(NULL);
			for (int n=5; n<20; ++n) {
				for (int i=0; i<5; ++i) {
					Graph g = *gen.createRandomGraph(n, max(n*(n-1)/4, n-1));
					int m = num_edges(g);

					//create variables
					vector<SOCMCrossingMinimization::crossingInfo> variableInfos 
						= cm.createVariables(g);
					vector<bool> variables(variableInfos.size());
					for (int j=0; j<variables.size(); ++j)
						variables[j] = false;

					//realize graph
					Graph g2 = cm.realize(g, variableInfos, variables);
					int n2 = num_vertices(g2);
					int m2 = num_edges(g2);

					//Test -> Graph should be equivalent
					Assert::AreEqual(n, n2, L"Node count does not match", LINE_INFO());
					Assert::AreEqual(m, m2, L"Edge count does not match", LINE_INFO());
				}
			}
		}

		TEST_METHOD(SOCM_TestRealize2) {
			GraphGenerator gen;
			SOCMCrossingMinimization cm(NULL);
			for (int n=5; n<20; ++n) {
				for (int i=0; i<5; ++i) {
					Graph g = *gen.createRandomGraph(n, max(n*(n-1)/4, n-1));
					int m = num_edges(g);

					//create variables
					vector<SOCMCrossingMinimization::crossingInfo> variableInfos 
						= cm.createVariables(g);
					vector<bool> variables(variableInfos.size());
					for (int j=0; j<variables.size(); ++j)
						variables[j] = false;

					//set some variables to true
					int crossings = 0;
					set<SOCMCrossingMinimization::edge> deletedEdges;
					for (int j=0; j<variables.size(); ++j) {
						SOCMCrossingMinimization::edge e = variableInfos[j].first;
						SOCMCrossingMinimization::edge f = variableInfos[j].second;
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
					Assert::AreEqual(expectedN, n2, L"Node count does not match", LINE_INFO());
					Assert::AreEqual(expectedM, m2, L"Edge count does not match", LINE_INFO());
				}
			}
		}

		TEST_METHOD(SOCM_TestRealize3) {
			//Make the K6 planar
			GraphGenerator gen;
			SOCMCrossingMinimization cm(NULL);
			Graph K6 = *gen.createRandomGraph(6, 6*(6-1)/2);
			vector<SOCMCrossingMinimization::crossingInfo> variableInfos = cm.createVariables(K6);
			vector<bool> variables(variableInfos.size());
			for (int j=0; j<variables.size(); ++j)
				variables[j] = false;
			Assert::IsFalse(boost::boyer_myrvold_planarity_test(K6), L"K6 should not be planar", LINE_INFO());
			
			//specify crossings between (0,1)x(3,5); (0,4)x(2,3); (1,2)x(4,5)
			int countOfOnes = 0;
			for (int i=0; i<variableInfos.size(); ++i) {
				SOCMCrossingMinimization::crossingInfo info = variableInfos[i];
				if (info == make_pair( make_pair(0,1), make_pair(3,5) )
					|| info == make_pair( make_pair(0,4), make_pair(2,3) )
					|| info == make_pair( make_pair(1,2), make_pair(4,5) )) {
						variables[i] = true;
						countOfOnes++;
				}
			}
			Assert::AreEqual(3, countOfOnes, L"Only three crossings should be set", LINE_INFO());

			//realize
			Graph g = cm.realize(K6, variableInfos, variables);
			Assert::IsTrue(boost::boyer_myrvold_planarity_test(g), L"K6 should now be planarized", LINE_INFO());
		}

		/*TEST_METHOD(TestMinimization) {
			GraphGenerator gen;
			MILP* milp = new MILP_lp_solve();
			CrossingMinimization* cm = new SOCMCrossingMinimization(milp);
			for (int n=5; n<20; ++n) {
				for (int i=0; i<5; ++i) {
					Graph g = *gen.createRandomGraph(n, max(n*(n-1)/6, n-1));
					int m = num_edges(g);

					boost::optional< pair<Graph, unsigned int> > result = cm->solve(g);
				}
			}
			delete cm;
			delete milp;
		}*/

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
#if 0
				//use planar layout
				ogdf::PlanarDrawLayout pdl;
				pdl.sideOptimization(true);
				pdl.sizeOptimization(true);
				pdl.call(GA);
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
			cout << "write graph to " << s.str() << endl;
			//Assert::Fail(ToString(s.str()).c_str());
		}

	};
}