#include "stdafx.h"
#include "CppUnitTest.h"
#include "CppUnitTestAssert.h"

#include <GraphGenerator.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace shaman;

namespace UnitTests
{		
	TEST_CLASS(TestGraphGenerator)
	{
	public:
		
		TEST_METHOD(TestWrongArguments)
		{
			GraphGenerator gen;
			// Tests that if the edge count is too high or too low, no graph is returned
			Assert::IsFalse(gen.createRandomGraph(0, 0));
			Assert::IsFalse(gen.createRandomGraph(1, 0));
			for (int n=2; n<50; ++n) {
				Assert::IsFalse(gen.createRandomGraph(n, n-2));
				Assert::IsTrue(gen.createRandomGraph(n, n-1));
				Assert::IsFalse(gen.createRandomGraph(n, n*(n-1)/2 + 1));
				Assert::IsTrue(gen.createRandomGraph(n, n*(n-1)/2));
			}
		}

	};
}