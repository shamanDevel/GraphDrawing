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

		GraphCopy createCopy(Graph& G) {
			GraphCopy C (G);
			return C;
		}
		TEST_METHOD(TestGraphCopy) {
			GraphGenerator gen;
			Graph G = *gen.createRandomGraph(10, 20);
			GraphCopy C = createCopy(G);
			//test if the linked graph is still the same
			node n1 = G.firstNode();
			node n2 = C.original(C.firstNode());
			node n3 = C.copy(n1);
			node n4 = C.firstNode();
			Assert::AreEqual(n1, n2, L"original node of the copy is not the same", LINE_INFO());
			Assert::AreEqual(n3, n4, L"copies are not the same", LINE_INFO());
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