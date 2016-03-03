#include "interpolation_term.h"
#include "solver.h"

InterpolationTerm::InterpolationTerm(int petal_id)
    :petal_id_(petal_id)
{

}

void InterpolationTerm::build()
{
    buildA();
    buildb();
}


// no projection needed
void InterpolationTerm::projection()
{
    return;
}

void InterpolationTerm::update()
{
    return;
}

void InterpolationTerm::buildA()
{
    Solver::DeformPetal& deform_petal = Solver::deform_petals_[petal_id_];
    Solver::ConvertAffineMatrix& convert_affine = deform_petal._convert_affine;
    int ver_num = deform_petal._petal_matrix.cols();

    L_.resize(3);
    A_.resize(3);

    std::vector<Eigen::Triplet<double> > diag_term;
    
    for (int i = 0; i < ver_num; ++i )
    {
        double wi = Solver::lambda_interpolation_;
        diag_term.push_back(Eigen::Triplet<double>(i, i, wi));
    }
    Eigen::SparseMatrix<double> diag_coeff(ver_num, ver_num);
    diag_coeff.setFromTriplets(diag_term.begin(), diag_term.end());

    // L for Vertice Variables
    L_[0] = diag_coeff;
    L_[1] = diag_coeff;
    L_[2] = diag_coeff;

    // A for Affine Transform Variables
    A_[0] = convert_affine.transpose() * (L_[0] * convert_affine);
    A_[1] = convert_affine.transpose() * (L_[1] * convert_affine);
    A_[2] = convert_affine.transpose() * (L_[2] * convert_affine);
}


void InterpolationTerm::buildb()
{
    Solver::DeformPetal& deform_petal = Solver::deform_petals_[petal_id_];
    Solver::PetalMatrix& ref_petal = deform_petal._ref_matrix;
    Solver::ConvertAffineMatrix& convert_affine = deform_petal._convert_affine;
    int ver_num = ref_petal.cols();

    b_.resize(3, ver_num);
    b_.setZero();
    for (int i = 0; i != ver_num; ++i )
    {
        b_.col(i) << Solver::lambda_interpolation_ * ref_petal.col(i);
    }

    // b_L_ for Vertex Variables
    b_L_ = b_;

    // for Affine Transform Variables
    b_ = b_ * convert_affine;

}

