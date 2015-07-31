#ifndef DATA_FITTING_TERM_H
#define DATA_FITTING_TERM_H

#include <Eigen/Sparse>

class Flower;
class PointCloud;

class DataFittingTerm
{
public:
    DataFittingTerm(int petal_id);
    inline std::vector<Eigen::MatrixXd>& A() { return A_; }
    inline Eigen::Matrix3Xd& b(){ return b_; }
    
    inline std::vector<Eigen::SparseMatrix<double>>& L() { return L_; }

    void build();
    void projection();
    void update();

protected:
    void buildA();
    void buildb();

    double zero_correction(double value);

private:
    int petal_id_;
    std::vector<Eigen::SparseMatrix<double>> L_;
    std::vector<Eigen::MatrixXd> A_;
    Eigen::Matrix3Xd b_;
};
#endif