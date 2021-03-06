#ifndef ARAP_TERM_H
#define ARAP_TERM_H

#include <Eigen/Sparse>

class Flower;
class PointCloud;

class ARAPTerm
{
public:
    ARAPTerm(int petal_id);
    inline std::vector<Eigen::MatrixXd>& A() { return A_; }
    inline Eigen::Matrix3Xd& b(){ return b_; }

    inline std::vector<Eigen::SparseMatrix<double>>& L(){ return L_; }
	inline Eigen::Matrix3Xd& b_L(){ return b_L_; }

    void build();
    void projection();
    void update();

protected:
    void buildA();
    void buildb();

    void initRotation();
    void updateRotation();

private:
    int petal_id_;
    std::vector<Eigen::SparseMatrix<double>> L_;
    std::vector<Eigen::MatrixXd> A_;
    Eigen::Matrix3Xd b_;
	Eigen::Matrix3Xd b_L_;

};
#endif