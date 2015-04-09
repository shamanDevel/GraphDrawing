#include "stdafx.h"
#include "CppUnitTest.h"
#include "CppUnitTestAssert.h"

#include <cstdlib>
#include <sstream>
#include <iostream>
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

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace shaman;
using namespace std;

//#define SAVE_GRAPHS

namespace UnitTests
{		
	TEST_CLASS(TestCrossingMinimization)
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

		TEST_METHOD(TestSplitGraph) {
			GraphGenerator gen;
			CrossingMinimization cm;
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