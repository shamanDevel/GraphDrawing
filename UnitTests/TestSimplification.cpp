#include "stdafx.h"
#include "CppUnitTest.h"
#include "CppUnitTestAssert.h"

#include <cstdlib>
#include <sstream>
#include <iostream>
#include <set>
#include <math.h>
#include <ogdf_include.h>
#include <GraphGenerator.h>
#include <GraphConverter.h>
#include <CrossingMinimization.h>
#include <SimplificationDeg12.h>
#include <ogdf/planarity/BoyerMyrvold.h>

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
		static Graph createRandomNonPlanarGraph(int n) {
			GraphGenerator gen;
			BoyerMyrvold bm;
			int m = n;
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

		TEST_METHOD(Test_SimplificationDeg12) {
			Graph G = createRandomNonPlanarGraph(10);

			const GraphCopy testCopy (G);
			AssertGraphEquality(G, testCopy);

			SimplificationDeg12 s (G);
			const GraphCopy& GC = s.getSimplifiedGraph();

			int ndif = G.numberOfNodes() - GC.numberOfNodes();
			Assert::IsTrue(ndif > 0, L"No nodes deleted", LINE_INFO());
			int mdif = G.numberOfEdges() - GC.numberOfEdges();
			Assert::AreEqual(ndif, mdif, L"Count of removed edges does not match count of removed nodes", LINE_INFO());

			GraphCopy G2 = s.reverseSimplification(GC);
			AssertGraphEquality(G, G2);
		}

		static int RandomInt(int min, int max) 
		{
			int r = rand();
			r = r % (max-min+1);
			r += min;
			return r;
		}

	};
}