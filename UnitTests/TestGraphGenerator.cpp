#include "stdafx.h"
#include "CppUnitTest.h"
#include "CppUnitTestAssert.h"

#include <cstdlib>
#include <GraphGenerator.h>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/boyer_myrvold_planar_test.hpp>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace shaman;
using namespace std;

namespace UnitTests
{		
	TEST_CLASS(TestGraphGenerator)
	{
	public:

		static int RandomInt(int min, int max) {
			int r = rand();
			r = r % (max-min+1);
			r += min;
			return r;
		}
		
		TEST_METHOD(TestWrongArguments)
		{
			GraphGenerator gen;
			// Tests that if the edge count is too high or too low, no graph is returned
			Assert::IsFalse(gen.createRandomGraph(0, 0), L"create empty graph", LINE_INFO());
			Assert::IsTrue(gen.createRandomGraph(1, 0), L"create one-node graph", LINE_INFO());
			for (int n=2; n<50; ++n) {
				Assert::IsFalse(gen.createRandomGraph(n, n-2), L"Create graph with too less edges", LINE_INFO());
				Assert::IsTrue(gen.createRandomGraph(n, n-1), L"Create line", LINE_INFO());
				Assert::IsFalse(gen.createRandomGraph(n, n*(n-1)/2 + 1), L"Create complete graph with one edge too much", LINE_INFO());
				Assert::IsTrue(gen.createRandomGraph(n, n*(n-1)/2), L"Create complete graph", LINE_INFO());
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
					Assert::IsTrue(og, L"Graph is created", LINE_INFO());
					Graph g = *og;
					int numNodes = num_vertices(g);
					int numEdges = num_edges(g);
					Assert::AreEqual(n, numNodes, L"Node count is correct", LINE_INFO());
					Assert::AreEqual(e, numEdges, L"Edge count is correct", LINE_INFO());
					vector<int> nodeVec(numNodes);
					int numComp = boost::connected_components(g, &nodeVec[0]);
					Assert::AreEqual(1, numComp, L"Only one connected component", LINE_INFO());
				}
			}
		}

		TEST_METHOD(TestPlanarGraphGen)
		{
			GraphGenerator gen;
			// Tests if the function createRandomPlanarGraph creates correct graphs
			for (int n=3; n<20; ++n) {
				for (int i=0; i<10; ++i) {
					int e = RandomInt(n*n-1, n*n*3);
					boost::optional<Graph> og = gen.createRandomPlanarGraph(n, e);
					Assert::IsTrue(og, L"Graph is created", LINE_INFO());
					Graph g = *og;
					int numNodes = num_vertices(g);
					int numEdges = num_edges(g);
					Assert::AreEqual(n*n, numNodes, L"Node count is correct", LINE_INFO());
					//Assert::AreEqual(e, numEdges, L"Edge count is correct", LINE_INFO());
					vector<int> nodeVec(numNodes);
					int numComp = boost::connected_components(g, &nodeVec[0]);
					Assert::AreEqual(1, numComp, L"Only one connected component", LINE_INFO());
					bool planar = boost::boyer_myrvold_planarity_test(g);
					Assert::IsTrue(planar, L"Graph is planar", LINE_INFO());
				}
			}
		}

	};
}