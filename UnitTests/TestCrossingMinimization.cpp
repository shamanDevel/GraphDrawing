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

		static int RandomInt(int min, int max) 
		{
			int r = rand();
			r = r % (max-min+1);
			r += min;
			return r;
		}

	};
}