#ifndef COLLISION_DETECTION_TERM_H
#define COLLISION_DETECTION_TERM_H

#include <Eigen/Sparse>

struct CollidingPoint;

class Flower;
class PointCloud;

struct CollisionPair
{
    CollisionPair(int vertex_id, osg::Vec3 projection){ vertex_id_ = vertex_id; projection_ = projection; }
    int vertex_id_;
    osg::Vec3 projection_;
};

class CollisionDetectionTerm
{
public:
	CollisionDetectionTerm(int petal_id );
    inline std::vector<Eigen::MatrixXd>& A() { return A_; }
    inline Eigen::Matrix3Xd& b(){ return b_; }

    inline std::vector<Eigen::SparseMatrix<double>>& L() { return L_; }
	inline Eigen::Matrix3Xd& b_L(){ return b_L_; }

    void build();
    void projection();
    void update();

protected:
    void buildA();
    void buildb();

    osg::Vec3 computeProjection(CollidingPoint cp);

    double zero_correction(double value);

private:
    int petal_id_;
    std::vector<Eigen::SparseMatrix<double>> L_;
    std::vector<Eigen::MatrixXd> A_;
    Eigen::Matrix3Xd b_;
	Eigen::Matrix3Xd b_L_;

    std::vector<CollisionPair>  collision_pair_;
};
#endif