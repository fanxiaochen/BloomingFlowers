#include "collision_detection_term.h"
#include "solver.h"

CollisionDetectionTerm::CollisionDetectionTerm(int petal_id)
    :petal_id_(petal_id)
{
}

void CollisionDetectionTerm::build()
{
    buildA();
    buildb();
}

void CollisionDetectionTerm::projection()
{
    collision_pair_.clear();
    Solver::IntersectList& intersect_list = Solver::deform_petals_[petal_id_]._intersect_list;
    for (int intersect_idx : intersect_list)
    {
        CollidingPoint cp = Solver::getCollidingPoint(petal_id_, intersect_idx);
        collision_pair_.push_back(CollisionPair(cp.vertex_id_ ,computeProjection(cp)));
        std::cout << cp.vertex_id_ << /*"  " << computeProjection(cp).x() << " " << computeProjection(cp).y() <<
                                      " " << computeProjection(cp).z();*/ std::endl;
    }

    return;
}

osg::Vec3 CollisionDetectionTerm::computeProjection(CollidingPoint cp)
{
    /*float a = cp.normal_.length2();
    float b = 2 * (cp.normal_ * cp.closest_p_);
    float c = cp.closest_p_.length2() - cp.dis2_;*/

    float k = sqrt(cp.dis2_ / cp.normal_.length2());

    osg::Vec3 projection;
    projection.x() = cp.normal_.x() * k + cp.closest_p_.x();
    projection.y() = cp.normal_.y() * k + cp.closest_p_.y();
    projection.z() = cp.normal_.z() * k + cp.closest_p_.z();

    /*std::cout << cp.normal_.x() << " " << cp.normal_.y() << " " << cp.normal_.z() << std::endl;
    std::cout << "a " << a << " b " << b << " c " << c << " k " << k << std::endl;
    std::cout << "x " << projection.x() << " y " << projection.y() << " z " << projection.z() << std::endl;*/
    return projection;
}

void CollisionDetectionTerm::update()
{
    buildA();
    buildb();
    return;
}

void CollisionDetectionTerm::buildA()
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
        diag_terms[0].push_back(Eigen::Triplet<double>(collision.vertex_id_, collision.vertex_id_, Solver::lambda_collision_));
        diag_terms[1].push_back(Eigen::Triplet<double>(collision.vertex_id_, collision.vertex_id_, Solver::lambda_collision_));
        diag_terms[2].push_back(Eigen::Triplet<double>(collision.vertex_id_, collision.vertex_id_, Solver::lambda_collision_));
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

void CollisionDetectionTerm::buildb()
{
    Solver::DeformPetal& deform_petal = Solver::deform_petals_[petal_id_];
    Solver::PetalMatrix& origin_petal = deform_petal._origin_petal;
    Solver::ConvertAffineMatrix& convert_affine = deform_petal._convert_affine;
    int ver_num = origin_petal.cols();

    b_.resize(3, ver_num);
    b_.setZero();

    for (auto& collision : collision_pair_)
    {
        b_.col(collision.vertex_id_) << Solver::lambda_collision_ * collision.projection_.x(), 
            Solver::lambda_collision_ * collision.projection_.y(), 
            Solver::lambda_collision_ * collision.projection_.z();
    }

    // for Affine Transform Variables
    b_ = b_ * convert_affine;
}

double CollisionDetectionTerm::zero_correction(double value)
{
    double min_double = std::numeric_limits<double>::min();
    if (min_double > value)
        return min_double;
    return value;
}