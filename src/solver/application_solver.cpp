#include "application_solver.h"

ApplicationSolver::ApplicationSolver(Flower* flower, int flower_frame)
	:Solver(NULL, flower, flower_frame)
{

}

void ApplicationSolver::init_setting()
{
	initMeshParas();
	std::cout << "finish mesh initialization" << std::endl;

	initTerms();
	std::cout << "finish solver initialization" << std::endl;
}

void ApplicationSolver::full_deform()
{
	std::cout << "Start application solver..." << std::endl;
	int iter = 0;
	double eps = 0;

	double e = 0;

	initBuild();
	do {
		collision_detection();
		projection();
		update();
		left_sys();
		right_sys();

		double e_n = solve();
		eps = std::fabs((e_n - e) / e_n);
		e = e_n;

		iter++;
	}while (eps > eps_ && iter < iter_num_);

	std::cout << "End application solver \t" << "iter: " << iter << "\tdelta: " << eps << std::endl;

	// flower deformed
	deforming();
}

void ApplicationSolver::initTerms()
{
	for (size_t i = 0, i_end = petal_num_; i < i_end; ++i)
	{
		arap_term_.push_back(ARAPTerm(i));
	}
	for (size_t i = 0, i_end = petal_num_; i < i_end; ++i)
	{
		collision_term_.push_back(CollisionDetectionTerm(i));
	}
	for (size_t i = 0, i_end = petal_num_; i < i_end; ++i)
	{
		interpolation_term_.push_back(InterpolationTerm(i));
	}
}

void ApplicationSolver::initBuild()
{
	for (size_t i = 0; i < petal_num_; ++i)
	{
		arap_term_[i].build();
		collision_term_[i].build();
		interpolation_term_[i].build();
	}
}
