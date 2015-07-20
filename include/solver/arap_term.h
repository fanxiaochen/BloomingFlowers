#ifndef ARAP_TERM_H
#define ARAP_TERM_H

#include <Eigen/Sparse>

class Flower;
class PointCloud;

class ARAPTerm
{
public:
    ARAPTerm(int petal_id);
    const Eigen::SparseMatrix<double>& const A(int i){ return A_[i]; }
    const Eigen::Matrix3Xd& const b(int i){ return b_.row(i); }

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