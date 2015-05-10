#include "stdafx.h"
#include "CppUnitTest.h"
#include "CppUnitTestAssert.h"
#include "TestUtilsInclude.h"

#include <cstdlib>
#include <sstream>
#include <iostream>
#include <set>
#include <math.h>
#include <ogdf_include.h>
#include <GraphGenerator.h>
#include <GraphConverter.h>
#include <CrossingMinimization.h>
#include <OOCMCrossingMinimization.h>
#include <SimplificationBiconnected.h>
#include <MILP_lp_solve.h>
#include <SimplificationDeg12.h>
#include <ogdf/planarity/BoyerMyrvold.h>
#include <ogdf/basic/simple_graph_alg.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace shaman;
using namespace std;
using namespace ogdf;

//#define SAVE_GRAPHS

namespace Microsoft{ namespace VisualStudio {namespace CppUnitTestFramework {

template<> static std::wstring ToString<ogdf::NodeElement> (ogdf::NodeElement* n) {
	if (n == NULL) {
		RETURN_WIDE_STRING("NULL");
	}
	RETURN_WIDE_STRING(n->index());
}

}}}

namespace UnitTests
{		
	TEST_CLASS(TestSimplification)
	{
	public:
#include "TestUtils.h"

		TEST_CLASS_INITIALIZE(Initialize)
		{
			RegisterDebugBoostSink();
		}

		static Graph createRandomNonPlanarGraph(int n) {
			GraphGenerator gen;
			BoyerMyrvold bm;
			int m = max(n, n*(n-1)/8);
			while (true) {
				Graph G = *gen.createRandomGraph(n, m);
				if (bm.isPlanar(G)) {
					m++;
				} else {
					return G;
				}
			}
		}

		static void AssertGraphEquality(const Graph& G, const GraphCopy& GC)
		{
			Assert::IsTrue(GC.consistencyCheck(), L"copied graph is not consistent", LINE_INFO());
			Assert::IsTrue(G.consistencyCheck(), L"original graph is not consistent", LINE_INFO());
			Assert::AreEqual(G.numberOfNodes(), GC.numberOfNodes(), L"wrong node count", LINE_INFO());
			Assert::AreEqual(G.numberOfEdges(), GC.numberOfEdges(), L"wrong edge count", LINE_INFO());
			node v;
			//check every node
			forall_nodes(v, GC) {
				node ov = GC.original(v);
				Assert::IsTrue(ov != NULL, L"copied node has no original node", LINE_INFO());
				//find node in source graph
				node u;
				node ou = NULL;
				forall_nodes(u, G) {
					if (u == ov) {
						Assert::IsTrue(ou == NULL, L"duplicate original nodes", LINE_INFO());
						ou = u;
					}
				}
				Assert::IsTrue(ou != NULL, L"original node of copied node is not in the original graph", LINE_INFO());
				//check adjacent edges
				Assert::AreEqual(ou->degree(), v->degree(), L"wrong node degree", LINE_INFO());
				adjEntry adj;
				forall_adj(adj, v) {
					node n = adj->twinNode();
					node on = GC.original(n);
					Assert::IsTrue(on != NULL, L"copied node has no original node", LINE_INFO());
					adjEntry adj2;
					bool found = false;
					forall_adj(adj2, ou) {
						node n2 = adj2->twinNode();
						if (n2 == on) {
							Assert::IsFalse(found, L"duplicate original nodes", LINE_INFO());
							found = true;
						}
					}
					Assert::IsTrue(found, L"no linked node in original node found", LINE_INFO());
				}
			}
		}

		TEST_METHOD(Test_SimplificationDeg12_BackForth) {
			Graph G = createRandomNonPlanarGraph(10);

			const GraphCopy testCopy (G);
			AssertGraphEquality(G, testCopy);

			SimplificationDeg12 s (G);
			const GraphCopy& GC = s.getSimplifiedGraph();

			int ndif = G.numberOfNodes() - GC.numberOfNodes();
			Assert::IsTrue(ndif > 0, L"No nodes deleted", LINE_INFO());

			GraphCopy G2 = s.reverseSimplification(GC);
			AssertGraphEquality(G, G2);
		}

		TEST_METHOD(Test_SimplificationDeg12_K5_1) {
			//Define a test graph that collapses to a K5
			Graph G;
			vector<node> nodes(41);
			for (int i=0; i<=40; ++i) nodes[i] = G.newNode();
			int edges[52][2] = {
				{0,7}, {0,6}, {0,9}, {0,18}, {0,19}, {0,35},
				{1,8}, {1,9}, {1,26}, {1,10}, {1,20}, {1,16}, {1,28},
				{2,11}, {2,18}, {2,12}, {2,22}, {2,25}, {2,17},
				{3,14}, {3,19}, {3,20}, {3,13}, {3,21}, {3,23},
				{4,35}, {4,5}, {4,16}, {4,17}, {4,15},
				{5,6}, {7,8}, {10,11}, {12,13}, {14,15},
				{21,22}, {23,24}, {24,25},
				{26,31}, {26,27}, {27,28}, {28,29}, {30,31}, {31,32},
				{33,34}, {33,35}, {34,35}, {33,36}, {33,37}, {37,38}, {37,39}, {37,40}
			};
			int edgeCosts[10][3] = {
				{0,1,2}, {0,2,1}, {0,3,1}, {0,4,2},
				{1,2,1}, {1,3,1}, {1,4,1},
				{2,3,3}, {2,4,1}, {3,4,1}
			};
			for (int i=0; i<52; ++i) {
				G.newEdge(nodes[edges[i][0]], nodes[edges[i][1]]);
			}

			SimplificationDeg12 s (G);
			const GraphCopy& GC = s.getSimplifiedGraph();
			const unordered_map<edge, int>& edgeCostMap = s.getEdgeCosts();
			const GraphCopy& GC2 (GC);

			//GC now should be a K5 with nodes 0..4
			Assert::AreEqual(5, GC2.numberOfNodes(), L"Wrong node count", LINE_INFO());
			Assert::AreEqual(10, GC2.numberOfEdges(), L"Wrong edge count", LINE_INFO());
			node u;
			forall_nodes(u, GC2) {
				Assert::AreEqual(4, u->degree());
			}
			Assert::IsTrue(isConnected(GC2));
			Assert::IsTrue(isSimpleUndirected(GC2));
			for (int i=0; i<10; ++i) {
				edge e = GC2.searchEdge(GC2.copy(nodes[edgeCosts[i][0]]), GC.copy(nodes[edgeCosts[i][1]]));
				Assert::IsNotNull(e);
				int costExp = edgeCosts[i][2];
				int costActual = edgeCostMap.at(GC2.original(e));
				Assert::AreEqual(costExp, costActual, L"Wrong edge cost", LINE_INFO());
			}

			const GraphCopy GC3 (GC2);
			GraphCopy GC4 = s.reverseSimplification(GC3);

			//Test it
			AssertGraphEquality(G, GC4);

			//Now solve the small graph
			OOCMCrossingMinimization cm (new MILP_lp_solve());
			CrossingMinimization::solve_result_t result = cm.solve(GC, edgeCostMap);
			Assert::IsTrue(result, L"Unable to solve K5", LINE_INFO());
			GraphCopy GC5 = result->first;

			//reverse simplification
			GraphCopy GC6 = s.reverseSimplification(GC5);
			BoyerMyrvold bm;
			Assert::IsTrue(bm.isPlanar(GC6));
		}

		TEST_METHOD(Test_SimplificationDeg12_K5_2) {
			//Define a test graph that collapses to a K5
			Graph G;
			vector<node> nodes(13);
			for (int i=0; i<=12; ++i) nodes[i] = G.newNode();
			int edges[21][2] = {
				{0,2}, {0,3}, {0,4}, {1,2}, {1,3}, {1,4}, {2,3}, {2,4}, {3,4},
				{0,5}, {0,6}, {5,7}, {6,7},
				{7,8}, {7,9}, {8,10}, {9,10},
				{10,11}, {10,12}, {11,1}, {12,1}
			};
			int edgeCosts[10][3] = {
				{0,1,2}, {0,2,1}, {0,3,1}, {0,4,1},
				{1,2,1}, {1,3,1}, {1,4,1},
				{2,3,1}, {2,4,1}, {3,4,1}
			};
			for (int i=0; i<21; ++i) {
				G.newEdge(nodes[edges[i][0]], nodes[edges[i][1]]);
			}
			SaveGraph(G, "BlownEdgeK5");

			SimplificationDeg12 s (G);
			const GraphCopy& GC = s.getSimplifiedGraph();
			const unordered_map<edge, int>& edgeCostMap = s.getEdgeCosts();
			const GraphCopy& GC2 (GC);
			SaveGraph(GC, "SimplifiedEdgeK5");

			//GC now should be a K5 with nodes 0..4
			Assert::AreEqual(5, GC2.numberOfNodes(), L"Wrong node count", LINE_INFO());
			Assert::AreEqual(10, GC2.numberOfEdges(), L"Wrong edge count", LINE_INFO());
			node u;
			forall_nodes(u, GC2) {
				Assert::AreEqual(4, u->degree());
			}
			Assert::IsTrue(isConnected(GC2));
			Assert::IsTrue(isSimpleUndirected(GC2));
			for (int i=0; i<10; ++i) {
				edge e = GC2.searchEdge(GC2.copy(nodes[edgeCosts[i][0]]), GC.copy(nodes[edgeCosts[i][1]]));
				Assert::IsNotNull(e);
				int costExp = edgeCosts[i][2];
				int costActual = edgeCostMap.at(GC2.original(e));
				Assert::AreEqual(costExp, costActual, L"Wrong edge cost", LINE_INFO());
			}

			const GraphCopy GC3 (GC2);
			GraphCopy GC4 = s.reverseSimplification(GC3);

			//Test it
			AssertGraphEquality(G, GC4);
		}

		TEST_METHOD(Test_SimplificationBiconnected) {
			BoyerMyrvold bm;
			for (int n=7; n<=20; ++n) {
				for (int i=0; i<5; ++i) {
					Graph G = createRandomNonPlanarGraph(n);
					Assert::IsFalse(bm.isPlanar(G));

					SimplificationBiconnected s (G);
					const vector<GraphCopy>& components = s.getComponents();
					for (const GraphCopy& GC : components) {
						Assert::IsFalse(bm.isPlanar(GC));
					}

					vector<GraphCopy> modifiedComponents(components.size());
					for (int j=0; j<components.size(); ++j) {
						modifiedComponents[j] = GraphCopy(components[j]);
					}

					GraphCopy G2 = s.reverseSimplification(modifiedComponents);

					AssertGraphEquality(G, G2);
				}
			}
		}
	};
}