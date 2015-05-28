#pragma once
#include <milp.h>
#include <lp_lib.h>

namespace shaman {

/// Implementation of MILP using lp_solve
class MILP_lp_solve :
	public MILP
{
public:
	MILP_lp_solve(void);
	virtual ~MILP_lp_solve(void);
	virtual bool initialize(unsigned int numVariables);
	virtual bool setVariableType(int variable, VariableType type);
	virtual bool addConstraint(int count, real* row, int* colno, ConstraintType type, real rhs);
	virtual bool setAddConstraintMode(bool mode);
	virtual bool setObjectiveFunction(int count, real* row, int* colno, Direction dir);
	virtual SolveResult solve(real* objective, real** variables);
	virtual void printDebug();
	virtual void setAbortFunction(abortfunc_t* abortFunction);

private:
	lprec *lp;
	abortfunc_t* abortFunction;
};

}