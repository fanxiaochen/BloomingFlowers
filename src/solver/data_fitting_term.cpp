#include "data_fitting_term.h"
#include "solver.h"

DataFittingTerm::DataFittingTerm(int petal_id)
    :petal_id_(petal_id)
{
}

void DataFittingTerm::build()
{
    buildA();
    buildb();
}

// no projection needed for data fitting
void DataFittingTerm::projection()
{
    return;
}

void DataFittingTerm::update()
{
    return;
}

void DataFittingTerm::buildA()
{
    Solver::DeformPetal& deform_petal = Solver::deform_petals_[petal_id_];
    Solver::CovMatrix& cov_matrix = deform_petal._cov_matrix;
    Solver::CorresMatrix& corres_matrix = deform_petal._corres_matrix;
    int ver_num = deform_petal._petal_matrix.cols();

    std::vector<std::vector<Eigen::Triplet<double> > > diag_terms;
    diag_terms.resize(3); // for x,y,z coordinates

    A_.resize(3);

    for (int i = 0; i < ver_num; ++i) 
    {
        double wi_x = 0, wi_y = 0, wi_z = 0;

        wi_x += zero_correction(Solver::lambda_*(2/cov_matrix.col(i)[0])*corres_matrix.row(i).sum());
        wi_y += zero_correction(Solver::lambda_*(2/cov_matrix.col(i)[1])*corres_matrix.row(i).sum());
        wi_z += zero_correction(Solver::lambda_*(2/cov_matrix.col(i)[2])*corres_matrix.row(i).sum());

        diag_terms[0].push_back(Eigen::Triplet<double>(i, i, wi_x));
        diag_terms[1].push_back(Eigen::Triplet<double>(i, i, wi_y));
        diag_terms[2].push_back(Eigen::Triplet<double>(i, i, wi_z));
    }

    Eigen::SparseMatrix<double> diag_coeff_x(ver_num, ver_num);
    Eigen::SparseMatrix<double> diag_coeff_y(ver_num, ver_num);
    Eigen::SparseMatrix<double> diag_coeff_z(ver_num, ver_num);
    diag_coeff_x.setFromTriplets(diag_terms[0].begin(), diag_terms[0].end());
    diag_coeff_y.setFromTriplets(diag_terms[1].begin(), diag_terms[1].end());
    diag_coeff_z.setFromTriplets(diag_terms[2].begin(), diag_terms[2].end());

    A_[0] = diag_coeff_x;
    A_[1] = diag_coeff_y;
    A_[2] = diag_coeff_z;
}

void DataFittingTerm::buildb()
{
    Solver::DeformPetal& deform_petal = Solver::deform_petals_[petal_id_];
    Solver::PetalMatrix& origin_petal = deform_petal._origin_petal;
    Solver::CloudMatrix& cloud_matrix = deform_petal._cloud_matrix;
    Solver::CorresMatrix& corres_matrix = deform_petal._corres_matrix;
    Solver::CovMatrix& cov_matrix = deform_petal._cov_matrix;
    int ver_num = origin_petal.cols();

    b_.resize(3, ver_num);

    for (size_t i = 0; i < ver_num; ++ i)
    {
        Eigen::Vector3d weight_cloud;
        weight_cloud.setZero();
        for (size_t n = 0, n_end = corres_matrix.cols(); n < n_end; ++ n)
        {
            weight_cloud += corres_matrix(i, n)*cloud_matrix.col(n);
        }

        b_.col(i) = Solver::lambda_*2*cov_matrix.col(i).asDiagonal().inverse()*weight_cloud;
    }
}

double DataFittingTerm::zero_correction(double value)
{
    double min_double = std::numeric_limits<double>::min();
    if (min_double > value)
        return min_double;
    return value;
}