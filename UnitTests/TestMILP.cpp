#include "stdafx.h"
#include "CppUnitTest.h"
#include "CppUnitTestAssert.h"

#include <cstdlib>
#include <sstream>
#include <iostream>
#include <math.h>
#include <MILP.h>
#include <MILP_lp_solve.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace shaman;
using namespace std;


namespace UnitTests
{		
	TEST_CLASS(TestMixedIntegerLinearProgramming)
	{
	public:

		TEST_METHOD(TestLinearProgramming)
		{
			MILP* lp = new MILP_lp_solve();
			Assert::IsTrue(lp->initialize(2), L"Unable to initialize LP solver", LINE_INFO());
			
			int *colno = NULL;
			MILP::real* row = NULL;
			colno = (int *) malloc(2 * sizeof(*colno));
			row = (REAL *) malloc(2 * sizeof(*row));
			colno[0] = 1;
			colno[1] = 2;

			//set objective function
			row[0] = 143;
			row[1] = 60;
			Assert::IsTrue(lp->setObjectiveFunction(2, row, colno, MILP::Direction::Maximize),
				L"Unable to set objective function", LINE_INFO());

			Assert::IsTrue(lp->setAddConstraintMode(true), L"Unable to switch to add-constraint-mode", LINE_INFO());
			//first row
			row[0] = 120;
			row[1] = 210;
			Assert::IsTrue(lp->addConstraint(2, row, colno, MILP::ConstraintType::LessThanEqual, 15000), 
				L"Unable to add first constraint", LINE_INFO());
			//second row
			row[0] = 110;
			row[1] = 30;
			Assert::IsTrue(lp->addConstraint(2, row, colno, MILP::ConstraintType::LessThanEqual, 4000), 
				L"Unable to add first constraint", LINE_INFO());
			//third row
			row[0] = 1;
			row[1] = 1;
			Assert::IsTrue(lp->addConstraint(2, row, colno, MILP::ConstraintType::LessThanEqual, 75), 
				L"Unable to add first constraint", LINE_INFO());
			Assert::IsTrue(lp->setAddConstraintMode(false), L"Unable to switch back to normal mode", LINE_INFO());

			//solve
			MILP::real objective;
			MILP::real* variables;
			MILP::SolveResult result = lp->solve(&objective, &variables);
			MILP::SolveResult expected = MILP::SolveResult::Optimal;
			Assert::AreEqual((int) expected, (int) result, L"Solver not returned the optimal solution", LINE_INFO());

			//test
			MILP::real expectedObjective = 6315.625;
			MILP::real* expectedVariables = new MILP::real[2];
			expectedVariables[0] = 21.875;
			expectedVariables[1] = 53.125;
			Assert::AreEqual(expectedObjective, objective, 0.001, L"Wrong Objective value", LINE_INFO());
			Assert::AreEqual(expectedVariables[0], variables[0], 0.001, L"Wrong x value", LINE_INFO());
			Assert::AreEqual(expectedVariables[1], variables[1], 0.001, L"Wrong y value", LINE_INFO());

			delete colno;
			delete row;
			delete lp;
			delete expectedVariables;
		}

		TEST_METHOD(TestIntegerLinearProgramming)
		{
			//same problems, now with integers

			MILP* lp = new MILP_lp_solve();
			Assert::IsTrue(lp->initialize(2), L"Unable to initialize LP solver", LINE_INFO());
			Assert::IsTrue(lp->setVariableType(1, MILP::VariableType::Integer), 
				L"Unable to change variable type to Integer", LINE_INFO());
			Assert::IsTrue(lp->setVariableType(2, MILP::VariableType::Integer), 
				L"Unable to change variable type to Integer", LINE_INFO());
			
			int *colno = NULL;
			MILP::real* row = NULL;
			colno = (int *) malloc(2 * sizeof(*colno));
			row = (REAL *) malloc(2 * sizeof(*row));
			colno[0] = 1;
			colno[1] = 2;

			//set objective function
			row[0] = 143;
			row[1] = 60;
			Assert::IsTrue(lp->setObjectiveFunction(2, row, colno, MILP::Direction::Maximize),
				L"Unable to set objective function", LINE_INFO());

			Assert::IsTrue(lp->setAddConstraintMode(true), L"Unable to switch to add-constraint-mode", LINE_INFO());
			//first row
			row[0] = 120;
			row[1] = 210;
			Assert::IsTrue(lp->addConstraint(2, row, colno, MILP::ConstraintType::LessThanEqual, 15000), 
				L"Unable to add first constraint", LINE_INFO());
			//second row
			row[0] = 110;
			row[1] = 30;
			Assert::IsTrue(lp->addConstraint(2, row, colno, MILP::ConstraintType::LessThanEqual, 4000), 
				L"Unable to add first constraint", LINE_INFO());
			//third row
			row[0] = 1;
			row[1] = 1;
			Assert::IsTrue(lp->addConstraint(2, row, colno, MILP::ConstraintType::LessThanEqual, 75), 
				L"Unable to add first constraint", LINE_INFO());
			Assert::IsTrue(lp->setAddConstraintMode(false), L"Unable to switch back to normal mode", LINE_INFO());

			//solve
			MILP::real objective;
			MILP::real* variables;
			MILP::SolveResult result = lp->solve(&objective, &variables);
			MILP::SolveResult expected = MILP::SolveResult::Optimal;
			Assert::AreEqual((int) expected, (int) result, L"Solver not returned the optimal solution", LINE_INFO());

			//stringstream s;
			//s << "objective value: " << objective << endl;
			//s << "x value: " << variables[0] << endl;
			//s << "y value: " << variables[1] << endl;
			//Logger::WriteMessage(s.str().c_str());

			//test
			MILP::real expectedObjective = 6266;
			MILP::real* expectedVariables = new MILP::real[2];
			expectedVariables[0] = 22;
			expectedVariables[1] = 52;
			Assert::AreEqual(expectedObjective, objective, 0, L"Wrong objective value", LINE_INFO());
			Assert::AreEqual(expectedVariables[0], variables[0], 0, L"Wrong x value", LINE_INFO());
			Assert::AreEqual(expectedVariables[1], variables[1], 0, L"Wrong y value", LINE_INFO());

			delete colno;
			delete row;
			delete lp;
			delete expectedVariables;
		}

	};
}