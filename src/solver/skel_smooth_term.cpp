#include "skel_smooth_term.h"
#include "solver.h"

SkelSmoothTerm::SkelSmoothTerm(int petal_id)
    :petal_id_(petal_id)
{
}

void SkelSmoothTerm::build()
{
    buildA();
    buildb();
}

// no projection needed for skeleton smooth
void SkelSmoothTerm::projection()
{
    return;
}

void SkelSmoothTerm::update()
{
    return;
}

void SkelSmoothTerm::buildA()
{
    Solver::DeformPetal& deform_petal = Solver::deform_petals_[petal_id_];
    Solver::HandleMatrix& handle_matrix = deform_petal._handle_matrix;
    int handle_number = handle_matrix.rows();
    A_.resize(3);

    // biharmonic weights for handles themselves
    Eigen::MatrixXd weight_matrix(handle_number, handle_number);
    weight_matrix.setIdentity();

    // handle coordinates
    Eigen::MatrixXd handles(handle_number, 4);
    handles.setOnes();
    handles.block(0, 0, handle_number, 3) = handle_matrix;

    // M(convert affine) for handles
    Eigen::MatrixXd M(handle_number, 4*handle_number);
    for (size_t j = 0, j_end = handle_number; j < j_end; ++ j)
    {
        M.block(0, 4*j, handle_number, 4) = weight_matrix.col(j).asDiagonal() * handles;
    }

    // A for relationship between neighbor handles
    Eigen::MatrixXd A(handle_number-2, handle_number);
    A.setZero();
    for (size_t i = 0; i < A.rows(); ++ i)
    {
        A(i,i) = 1;
        A(i, i+1) = -2;
        A(i, i+2) = 1;
    }

    // lambda_skel_smooth matrix
    Eigen::MatrixXd lambda_ss(handle_number-2, handle_number-2);
    lambda_ss.setIdentity();
    lambda_ss = Solver::lambda_skel_smooth_ * lambda_ss;

    A_[0] = 2 * M.transpose() * A.transpose() * lambda_ss * A * M;
    A_[1] = 2 * M.transpose() * A.transpose() * lambda_ss * A * M;
    A_[2] = 2 * M.transpose() * A.transpose() * lambda_ss * A * M;
}

void SkelSmoothTerm::buildb()
{
    Solver::DeformPetal& deform_petal = Solver::deform_petals_[petal_id_];
    Solver::HandleMatrix& handle_matrix = deform_petal._handle_matrix;
    int handle_number = handle_matrix.rows();

    b_.resize(3, 4*handle_number);
    b_.setZero();
}

double SkelSmoothTerm::zero_correction(double value)
{
    double min_double = std::numeric_limits<double>::min();
    if (min_double > value)
        return min_double;
    return value;
}