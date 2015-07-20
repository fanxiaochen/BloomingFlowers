#ifndef ARAP_TERM_H
#define ARAP_TERM_H

#include <Eigen/Sparse>

class Flower;
class PointCloud;

class ARAPTerm
{
public:
    ARAPTerm(int petal_id);
    inline std::vector<Eigen::SparseMatrix<double>>& A(){ return A_; }
    inline Eigen::Matrix3Xd& b(){ return b_; }

    void init();
    void projection();
    void update();

protected:
    void buildA();
    void buildb();

    void initRotation();
    void updateRotation();

private:
    int petal_id_;
    std::vector<Eigen::SparseMatrix<double>> A_;
    Eigen::Matrix3Xd b_;
};
#endif