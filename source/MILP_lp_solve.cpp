#include "stdafx.h"
#include <MILP_lp_solve.h>

namespace shaman {

MILP_lp_solve::MILP_lp_solve(void)
	: lp(NULL)
{
}


MILP_lp_solve::~MILP_lp_solve(void)
{
	if (lp!=NULL) {
		delete_lp(lp);
		lp = NULL;
	}
}

bool MILP_lp_solve::initialize(unsigned int numVariables)
{
	if (lp!=NULL) {
		delete_lp(lp);
	}
	lp = make_lp(0, numVariables);
	return lp != NULL;
}

bool MILP_lp_solve::setVariableType(int variable, VariableType type)
{
	switch (type)
	{
	case VariableType::Real:
		return set_int(lp, variable, 0);
	case VariableType::Integer:
		return set_int(lp, variable, 1);
	case VariableType::Binary:
		return set_binary(lp, variable, 1);
	}
	return false;
}
	
bool MILP_lp_solve::addConstraint(int count, real* row, int* colno, ConstraintType type, real rhs)
{
	int constr_type = 0;
	switch (type)
	{
	case ConstraintType::LessThanEqual:
		constr_type = LE;
		break;
	case ConstraintType::Equal:
		constr_type = EQ;
		break;
	case ConstraintType::GreaterThanEqual:
		constr_type = GE;
		break;
	}
	return add_constraintex(lp, count, row, colno, constr_type, rhs);
}

bool MILP_lp_solve::setAddConstraintMode(bool mode) 
{
	return set_add_rowmode(lp, mode);
}

bool MILP_lp_solve::setObjectiveFunction(int count, real* row, int* colno, Direction dir)
{
	if (!set_obj_fnex(lp, count, row, colno))
		return false;
	if (dir == Direction::Maximize)
		set_maxim(lp);
	else
		set_minim(lp);
	return true;
}

MILP_lp_solve::SolveResult MILP_lp_solve::solve(real* objective, real** variables) 
{
	int ret = ::solve(lp);
	if (ret == OPTIMAL) {
		//optimal solution, collect results
		*objective = get_objective(lp);
		get_ptr_variables(lp, variables);
		return SolveResult::Optimal;
	} else {
		//error codes
		switch (ret) {
		case NOMEMORY: return SolveResult::OutOfMemory;
		case SUBOPTIMAL: return SolveResult::Suboptimal;
		case INFEASIBLE: return SolveResult::Infeasible;
		case DEGENERATE: return SolveResult::Degenerate;
		case UNBOUNDED: return SolveResult::Unbounded;
		default: return SolveResult::Other;
		}
	}
}

}