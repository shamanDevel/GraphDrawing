#include "stdafx.h"
#include "CppUnitTest.h"
#include "CppUnitTestAssert.h"

#include <cstdlib>
#include <sstream>
#include <iostream>
#include <math.h>
#include <GraphGenerator.h>
#include <ogdf_include.h>
#include <ogdf\planarlayout\PlanarDrawLayout.h>
#include <ogdf\energybased\FMMMLayout.h>
#include <ogdf\energybased\StressMajorizationSimple.h>
#include <ogdf\planarity\BoyerMyrvold.h>
#include <ogdf\basic\simple_graph_alg.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace shaman;
using namespace std;
using namespace ogdf;

//#define SAVE_GRAPHS

namespace UnitTests
{		
	TEST_CLASS(TestGraphGenerator)
	{
	public:

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
#endif
		}
		
		TEST_METHOD(TestWrongArguments)
		{
			GraphGenerator gen;
			// Tests that if the edge count is too high or too low, no graph is returned
			Assert::IsFalse(gen.createRandomGraph(0, 0), L"unable to create empty graph", LINE_INFO());
			Assert::IsTrue(gen.createRandomGraph(1, 0), L"unable to create one-node graph", LINE_INFO());
			for (int n=2; n<50; ++n) {
				Assert::IsFalse(gen.createRandomGraph(n, n-2), L"graph with too less edges created", LINE_INFO());
				Assert::IsTrue(gen.createRandomGraph(n, n-1), L"unable to create line", LINE_INFO());
				Assert::IsFalse(gen.createRandomGraph(n, n*(n-1)/2 + 1), L"graph with one edge too much created", LINE_INFO());
				Assert::IsTrue(gen.createRandomGraph(n, n*(n-1)/2), L"unable to create complete graph", LINE_INFO());
			}
		}

		TEST_METHOD(TestGraphGen)
		{
			GraphGenerator gen;
			// Tests if the function createRandomGraph creates correct graphs
			for (int n=10; n<50; ++n) {
				for (int i=0; i<10; ++i) {
					int e = RandomInt(n-1, n*(n-1)/2);
					boost::optional<Graph> og = gen.createRandomGraph(n, e);
					Assert::IsTrue(og, L"unable to create graph", LINE_INFO());
					Graph g = *og;
					int numNodes = g.numberOfNodes();
					int numEdges = g.numberOfEdges();
					Assert::AreEqual(n, numNodes, L"Node count is wrong", LINE_INFO());
					Assert::AreEqual(e, numEdges, L"Edge count is wrong", LINE_INFO());
					bool connected = isConnected(g);
					Assert::IsTrue(connected, L"Graph is not connected", LINE_INFO());

					SaveGraph(g, "random");
				}
			}
		}

		TEST_METHOD(TestPlanarGraphGen)
		{
			GraphGenerator gen;
			BoyerMyrvold bm;
			// Tests if the function createRandomPlanarGraph creates correct graphs
			for (int n=3; n<20; ++n) {
				for (int i=0; i<10; ++i) {
					int e;
					if (i==0)
						e = n*n-1; //lower bound -> tree
					else if (i==1)
						e = (n-1)*(n-1)*3 + (n-1)*2; //upper bound
					else
						e = RandomInt(n*n, (n-1)*(n-1)*3 + (n-1)*2 - 1);
					boost::optional<Graph> og = gen.createRandomPlanarGraph(n, e);
					Assert::IsTrue(og, L"unable to create graph", LINE_INFO());
					Graph g = *og;
					int numNodes = g.numberOfNodes();
					int numEdges = g.numberOfEdges();
					Assert::AreEqual(n*n, numNodes, L"Node count is wrong", LINE_INFO());
					Assert::AreEqual(e, numEdges, L"Edge count is wrong", LINE_INFO());
					bool connected = isConnected(g);
					Assert::IsTrue(connected, L"graph is not connected", LINE_INFO());
					bool planar = bm.isPlanar(g);
					Assert::IsTrue(planar, L"Graph is not planar", LINE_INFO());

					SaveGraph(g, "randomPlanar");
				}
			}
		}

	};
}