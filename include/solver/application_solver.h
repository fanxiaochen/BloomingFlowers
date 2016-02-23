#ifndef APPLICATION_SOLVER_H
#define APPLICATION_SOLVER_H

#include "solver.h"
#include "interpolation_term.h"
//////////////////////////////////////////////////////////////////////////
// to address the new collision introduced by application: 
// use interpolated positions as soft constraints, and solve for
// shape term and penetration avoidance term to find the optimal positions.
//////////////////////////////////////////////////////////////////////////
class ApplicationSolver : public Solver
{
public:
	ApplicationSolver(Flower* flower, int flower_frame);
	void init_setting();   // rewrite the init setting function
	void full_deform();    // rewrite the full deform function

protected:
	void initTerms();    // rewrite the virtual function
	void initBuild();    // rewrite the init build function

protected:
	std::vector<InterpolationTerm> interpolation_term_;
};

#endif 

