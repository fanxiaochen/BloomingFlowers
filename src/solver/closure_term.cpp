#include "closure_term.h"
#include "solver.h"
#include "main_window.h"
#include "parameters.h"

ClosureTerm::ClosureTerm(int petal_id)
    :petal_id_(petal_id)
{
}

void ClosureTerm::build()
{
    buildA();
    buildb();
}

void ClosureTerm::projection()
{
    const std::string& valid_ids = Solver::closure_ids_;
    int i = 0;
    for (; i < valid_ids.size(); i++)
        if ((valid_ids[i] - '0') == petal_id_)
            break;
    if (i == valid_ids.size())
        return;

    const int& closure_start_frame = Solver::closure_start_frame_;
    if (closure_start_frame < Solver::cloud_frame_)
        return;

    collision_pair_.clear();

    pcl::KdTreeFLANN<Point>& kdtree = Solver::closure_cloud_->getSelfKdtree();

    Solver::PetalMatrix& petal_matrix = Solver::deform_petals_[petal_id_]._petal_matrix;
    
    for (int i = 0, i_end = petal_matrix.cols(); i < i_end; ++ i)
    {
        Point search_point;
        search_point.x = petal_matrix(0, i);
        search_point.y = petal_matrix(1, i);
        search_point.z = petal_matrix(2, i);
        int K = 3;

        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        if ( kdtree.nearestKSearch (search_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            Point p;
            osg::Vec3 cloud_normal;
            for (int index : pointIdxNKNSearch)
            {
                const Point& point = Solver::closure_cloud_->at(index);
                p = p + point;
                cloud_normal += osg::Vec3(point.normal_x, point.normal_y, point.normal_z);
            }

            p = p / pointIdxNKNSearch.size();
            cloud_normal = cloud_normal / pointIdxNKNSearch.size();
            cloud_normal.normalize();

            osg::Vec3 p_normal(p.x - search_point.x, p.y - search_point.y, p.z - search_point.z);
            p_normal.normalize();

            if (p_normal * cloud_normal < 0)
                collision_pair_.push_back(CollisionPair(i , osg::Vec3(p.x, p.y, p.z)));
        }
    }

    return;
}

void ClosureTerm::update()
{
    const std::string& valid_ids = Solver::closure_ids_;
    int i = 0;
    for (; i < valid_ids.size(); i++)
        if ((valid_ids[i] - '0') == petal_id_)
            break;
    if (i == valid_ids.size())
        return;

    const int& closure_start_frame = Solver::closure_start_frame_;
    if (closure_start_frame < Solver::cloud_frame_)
        return;


    buildA();
    buildb();
    return;
}

void ClosureTerm::buildA()
{
    Solver::DeformPetal& deform_petal = Solver::deform_petals_[petal_id_];
    Solver::ConvertAffineMatrix& convert_affine = deform_petal._convert_affine;
    int ver_num = deform_petal._petal_matrix.cols();

    std::vector<std::vector<Eigen::Triplet<double> > > diag_terms;
    diag_terms.resize(3); // for x,y,z coordinates

    L_.resize(3);
    A_.resize(3);

    for (auto& collision : collision_pair_)
    {
        diag_terms[0].push_back(Eigen::Triplet<double>(collision.vertex_id_, collision.vertex_id_, Solver::lambda_closure_));
        diag_terms[1].push_back(Eigen::Triplet<double>(collision.vertex_id_, collision.vertex_id_, Solver::lambda_closure_));
        diag_terms[2].push_back(Eigen::Triplet<double>(collision.vertex_id_, collision.vertex_id_, Solver::lambda_closure_));
    }

    Eigen::SparseMatrix<double> diag_coeff_x(ver_num, ver_num);
    Eigen::SparseMatrix<double> diag_coeff_y(ver_num, ver_num);
    Eigen::SparseMatrix<double> diag_coeff_z(ver_num, ver_num);
    diag_coeff_x.setFromTriplets(diag_terms[0].begin(), diag_terms[0].end());
    diag_coeff_y.setFromTriplets(diag_terms[1].begin(), diag_terms[1].end());
    diag_coeff_z.setFromTriplets(diag_terms[2].begin(), diag_terms[2].end());

    // L for Vertice Variables
    L_[0] = diag_coeff_x;
    L_[1] = diag_coeff_y;
    L_[2] = diag_coeff_z;

    // A for Affine Transform Variables
    A_[0] = convert_affine.transpose() * (L_[0] * convert_affine);
    A_[1] = convert_affine.transpose() * (L_[1] * convert_affine);
    A_[2] = convert_affine.transpose() * (L_[2] * convert_affine);

}

void ClosureTerm::buildb()
{
    Solver::DeformPetal& deform_petal = Solver::deform_petals_[petal_id_];
    Solver::PetalMatrix& origin_petal = deform_petal._origin_petal;
    Solver::ConvertAffineMatrix& convert_affine = deform_petal._convert_affine;
    int ver_num = origin_petal.cols();

    b_.resize(3, ver_num);
    b_.setZero();

    for (auto& collision : collision_pair_)
    {
        b_.col(collision.vertex_id_) << Solver::lambda_closure_ * collision.projection_.x(), 
            Solver::lambda_closure_ * collision.projection_.y(), 
            Solver::lambda_closure_ * collision.projection_.z();
    }

    b_L_ = b_;

    // for Affine Transform Variables
    b_ = b_ * convert_affine;
}

double ClosureTerm::zero_correction(double value)
{
    double min_double = std::numeric_limits<double>::min();
    if (min_double > value)
        return min_double;
    return value;
}