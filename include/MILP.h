#pragma once

#include <sstream>

namespace shaman {

/// Base interface for Mixed-Integer-Linear-Program Solvers.
/// The variable indices start at 1 !
class MILP
{
public:
	typedef double real;

	MILP(void)
	{
	}

	virtual ~MILP(void)
	{
	}

	///	\brief	Initializes this LP-model with the specified count of variables.
	///
	///	\param numVariables	The count of variables
	virtual bool initialize(unsigned int numVariables) = 0;

	static enum VariableType {
		Real, Integer, Binary
	};

	///	\brief	Sets the type of the specified variable.
	///			By default, all variables are reals.
	///
	///	\param variable	The index (beginning with 1) of the variable
	///	\param type		The type of the variable.
	///	\return true if successfull, false if an error occured
	virtual bool setVariableType(int variable, VariableType type) = 0;

	static enum ConstraintType {
		LessThanEqual,
		Equal,
		GreaterThanEqual
	};

	///	\brief	Adds a new row to the LP-model
	///			The row contents are described by the two arrays colno and row of length count.
	///			colno specifies the index (1-based) of the variables and row then contains the factor for that variable.
	///			rhs finally contains the value on the right side of the equation.
	///
	///	\param count	The count of variables
	///	\param row		The factors for the variables
	///	\param colno	Specifies which variables are used (1-based)
	///	\param type		The type of constraint (<=, ==, >=)
	///	\param rhs		The value of the right hand side
	///	\return true if successfull, false if an error occured
	virtual bool addConstraint(int count, real* row, int* colno, ConstraintType type, real rhs) = 0;

	///	\brief	If this flag is set to true, the addConstraint method performs faster.
	///			Don't forget to set this flag back to false before solving the model.
	///
	///	\param mode	true to increase speed of addConstraint, call false before solving it.
	///	\return true if successfull, false if an error occured
	virtual bool setAddConstraintMode(bool mode) {return true;}

	static enum Direction {
		Minimize, Maximize
	};

	///	\brief	Sets the objective function.
	///			The row contents are described by the two arrays colno and row of length count.
	///			colno specifies the index (1-based) of the variables and row then contains the factor for that variable.
	///			Is is strongly advised to call this method before adding constraint (better performance)
	///
	///	\param count	The count of variables
	///	\param row		The factors for the variables
	///	\param colno	Specifies which variables are used (1-based)
	///	\param dir		The direction of the optimization (minimize or maximize)
	///	\return true if successfull, false if an error occured
	virtual bool setObjectiveFunction(int count, real* row, int* colno, Direction dir) = 0;

	static enum SolveResult {
		Optimal,
		Suboptimal,
		OutOfMemory,
		Infeasible,
		Unbounded,
		Degenerate,
		Other
	};

	///	\brief	Solves the model.
	///			The result values 'objective' and 'variables' are only valid if the solver returns SolveResult::Optimal.
	///
	///	\param objective	This variable is filled with the value of the objective function.
	///	\param variables	The address of a pointer that will point to an array that will contain the values of the variables.
	///						Element 0 will contain the value of the first variable, element 1 of the second variable, ...
	///						DON'T FREE THIS MEMORY!
	///	\return	An enum indicating the state of the solution
	virtual SolveResult solve(real* objective, real** variables) = 0;

	///	\brief	Prints debug information about the model to stdout.
	virtual void printDebug() {}
};

}