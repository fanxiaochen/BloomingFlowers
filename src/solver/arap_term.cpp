#include "arap_term.h"
#include "solver.h"

#include "WunderSVD3x3.h"

ARAPTerm::ARAPTerm(int petal_id)
    :petal_id_(petal_id)
{
}

void ARAPTerm::build()
{
    initRotation();
    buildA();
    buildb();
}

void ARAPTerm::projection()
{
    updateRotation();
    return;
}

// only need to update b
void ARAPTerm::update()
{
    buildb();
}

void ARAPTerm::buildA()
{
    Solver::DeformPetal& deform_petal = Solver::deform_petals_[petal_id_];
    Solver::CovMatrix& cov_matrix = deform_petal._cov_matrix;
    Solver::WeightMatrix& weight_matrix = deform_petal._weight_matrix;
    Solver::AdjList& adj_list = deform_petal._adj_list;
    Solver::ConvertAffineMatrix& convert_affine = deform_petal._convert_affine;
    Solver::HardCtrsIdx& hc_idx = deform_petal._hc_idx;
    int ver_num = deform_petal._petal_matrix.cols();

    std::vector<std::vector<Eigen::Triplet<double> > > weight_sums;
    weight_sums.resize(3); // for x,y,z coordinates

    L_.resize(3);
    A_.resize(3);

    for (int i = 0; i < ver_num; ++i) 
    {
        double wi_x = 0, wi_y = 0, wi_z = 0;

        for (size_t j = 0, j_end = deform_petal._adj_list[i].size(); j < j_end; ++j)
        {
            int id_j = deform_petal._adj_list[i][j];
            wi_x += weight_matrix.coeffRef(i, id_j);
            wi_y += weight_matrix.coeffRef(i, id_j);
            wi_z += weight_matrix.coeffRef(i, id_j);
        }

        weight_sums[0].push_back(Eigen::Triplet<double>(i, i, wi_x));
        weight_sums[1].push_back(Eigen::Triplet<double>(i, i, wi_y));
        weight_sums[2].push_back(Eigen::Triplet<double>(i, i, wi_z));
    }

    Eigen::SparseMatrix<double> diag_coeff_x(ver_num, ver_num);
    Eigen::SparseMatrix<double> diag_coeff_y(ver_num, ver_num);
    Eigen::SparseMatrix<double> diag_coeff_z(ver_num, ver_num);
    diag_coeff_x.setFromTriplets(weight_sums[0].begin(), weight_sums[0].end());
    diag_coeff_y.setFromTriplets(weight_sums[1].begin(), weight_sums[1].end());
    diag_coeff_z.setFromTriplets(weight_sums[2].begin(), weight_sums[2].end());

    // L for Vertice Variables
    L_[0] = diag_coeff_x - weight_matrix;
    L_[1] = diag_coeff_y - weight_matrix;
    L_[2] = diag_coeff_z - weight_matrix;

    // hard constraint
    for (int idx: hc_idx)
    {
        for (int col = 0; col < ver_num; ++ col)
        {
            if (col == idx)
            {
                L_[0].coeffRef(idx, idx) = 1;
                L_[1].coeffRef(idx, idx) = 1;
                L_[2].coeffRef(idx, idx) = 1;
            }
            else
            {
                L_[0].coeffRef(idx, col) = 0;
                L_[1].coeffRef(idx, col) = 0;
                L_[2].coeffRef(idx, col) = 0;
            }
        }
    }

    // A for Affine Transforms Variables
    A_[0] = convert_affine.transpose() * (L_[0] * convert_affine);
    A_[1] = convert_affine.transpose() * (L_[1] * convert_affine);
    A_[2] = convert_affine.transpose() * (L_[2] * convert_affine);
}

void ARAPTerm::buildb()
{
    Solver::DeformPetal& deform_petal = Solver::deform_petals_[petal_id_];
    Solver::PetalMatrix& origin_petal = deform_petal._origin_petal;
    Solver::AdjList& adj_list = deform_petal._adj_list;
    Solver::WeightMatrix& weight_matrix = deform_petal._weight_matrix;
    Solver::RotList& R_list = deform_petal._R_list;
    Solver::ScaleList& S_list = deform_petal._S_list;
    Solver::ConvertAffineMatrix& convert_affine = deform_petal._convert_affine;
    Solver::HardCtrsIdx& hc_idx = deform_petal._hc_idx;
    int ver_num = origin_petal.cols();

    b_.resize(3, ver_num);

    for (size_t i = 0; i < ver_num; ++i) 
    {
        b_.col(i) = Eigen::Vector3d::Zero();
        for (size_t j = 0, j_end = adj_list[i].size(); j < j_end; ++j)
        {
            b_.col(i) += ((weight_matrix.coeffRef(i, adj_list[i][j])/2)*
                (S_list[i]*R_list[i]+S_list[adj_list[i][j]]*R_list[adj_list[i][j]])*(origin_petal.col(i) - origin_petal.col(adj_list[i][j]))).transpose();
        }
    }

    // hard constraint
    for (int idx : hc_idx)
    {
        b_.col(idx) << origin_petal.col(idx);
    }

    //for Affine Transform Variables
    b_ = b_ * convert_affine;
}

void ARAPTerm::initRotation()
{
    // init rotation matrix and scale list
    Solver::DeformPetal& deform_petal = Solver::deform_petals_[petal_id_];
    Solver::PetalMatrix& origin_petal = deform_petal._origin_petal;
    Solver::RotList R_list;

    for (size_t j = 0, j_end = origin_petal.cols(); j < j_end; ++ j)
    {
        R_list.push_back(Eigen::Matrix3d::Identity());
    }

    deform_petal._R_list = R_list;

    deform_petal._S_list = std::vector<double>(origin_petal.cols(), 1);
}

void ARAPTerm::updateRotation()
{
    Solver::DeformPetal& deform_petal = Solver::deform_petals_[petal_id_];
    Solver::PetalMatrix& origin_petal = deform_petal._origin_petal;
    Solver::PetalMatrix& petal_matrix = deform_petal._petal_matrix;
    Solver::RotList& rot_list = deform_petal._R_list;
    Solver::ScaleList& scale_list = deform_petal._S_list;
    Solver::AdjList& adj_list = deform_petal._adj_list;
    Solver::WeightMatrix& weight_matrix = deform_petal._weight_matrix;

    Eigen::Matrix3f Si;
    Eigen::MatrixXd Di;

    Eigen::Matrix3Xd Pi_Prime;
    Eigen::Matrix3Xd Pi;

    for (size_t i = 0, i_end = rot_list.size(); i < i_end; ++i) 
    {
        Di = Eigen::MatrixXd::Zero(adj_list[i].size(), adj_list[i].size());
        Pi_Prime.resize(3, adj_list[i].size());
        Pi.resize(3, adj_list[i].size());

        for (size_t j = 0, j_end = adj_list[i].size(); j < j_end; ++j) 
        {
            Di(j, j) = weight_matrix.coeffRef(i, adj_list[i][j]);
            Pi.col(j) = origin_petal.col(i) - origin_petal.col(adj_list[i][j]);
            Pi_Prime.col(j) = petal_matrix.col(i) - petal_matrix.col(adj_list[i][j]);
        }
        Si = Pi.cast<float>() * Di.cast<float>() * Pi_Prime.transpose().cast<float>();
        Eigen::Matrix3f Ui;
        Eigen::Vector3f Wi;
        Eigen::Matrix3f Vi;
        wunderSVD3x3(Si, Ui, Wi, Vi);
        rot_list[i] = Vi.cast<double>() * Ui.transpose().cast<double>();

        if (rot_list[i].determinant() < 0)
            std::cout << "determinant is negative!" << std::endl;

        double s = 0;
        for (size_t j = 0, j_end = adj_list[i].size(); j < j_end; ++j) 
        {
            s += Di(j, j) * Pi.col(j).squaredNorm();
        }

       // scale_list[i] = Wi.trace() / s;

       /* if (scale_list[i] < 0.95 )
            scale_list[i] = 0.95;
        else if (scale_list[i] > 1.05)
            scale_list[i] = 1.05;*/
    }

    /*if (petal_id_ == 0) std::cout << "vertex: " << 0 << "  " << scale_list[0] << std::endl;*/
}



