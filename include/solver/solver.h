#ifndef SOLVER_H
#define SOLVER_H

#include <vector>

#include <Eigen/Sparse>

#include "data_fitting_term.h"
#include "arap_term.h"
#include "skel_smooth_term.h"

class Flower;
class PointCloud;

class Solver
{
public:
    typedef Eigen::MatrixXd CorresMatrix;
    typedef Eigen::Matrix3Xd CloudMatrix;
    typedef Eigen::Matrix3Xd PetalMatrix;
    typedef Eigen::Matrix3Xd CovMatrix;
    typedef std::vector<double> WeightList;
    typedef std::vector<std::vector<int> > AdjList;
    typedef std::vector<Eigen::Vector3i > FaceList;
    typedef Eigen::SparseMatrix<double> WeightMatrix;
    typedef std::vector<Eigen::Matrix3d> RotList;
    typedef Eigen::MatrixXd BiWeightMatrix;
    typedef Eigen::MatrixXd ConvertAffineMatrix;
    typedef Eigen::MatrixXd AffineMatrix;
    typedef Eigen::MatrixXd HandleMatrix;
    typedef std::vector<std::vector<int>> BranchList;

public:
    typedef struct 
    {
        PetalMatrix         _origin_petal;
        PetalMatrix         _petal_matrix;
        CloudMatrix         _cloud_matrix;
        CorresMatrix        _corres_matrix;
        CovMatrix           _cov_matrix;
        WeightList          _weight_list;
        AdjList             _adj_list;
        FaceList            _face_list;
        WeightMatrix        _weight_matrix;
        RotList             _R_list;
        BiWeightMatrix      _biweight_matrix;
        ConvertAffineMatrix _convert_affine;
        AffineMatrix        _affine_matrix;
        HandleMatrix        _handle_matrix;
        BranchList          _branch_list;

        void findSharedVertex(int pi, int pj, std::vector<int>& share_vertex)
        {
            std::vector<int> vertices;
            set_intersection(_adj_list[pi].begin(), _adj_list[pi].end(), _adj_list[pj].begin(), _adj_list[pj].end(), back_inserter(vertices));
            for (auto &i : vertices) 
            {
                std::vector<int> f;
                f.push_back(pi);
                f.push_back(pj);
                f.push_back(i);
                sort(f.begin(), f.end());
                std::vector<Eigen::Vector3i>::iterator it = std::find(_face_list.begin(), _face_list.end(), Eigen::Map<Eigen::Vector3i>(&f[0]));
                if (it != _face_list.end()) {
                    if ((*it)(0) != pi && (*it)(0) != pj) share_vertex.push_back((*it)(0));
                    else if ((*it)(1) != pi && (*it)(1) != pj) share_vertex.push_back((*it)(1));
                    else share_vertex.push_back((*it)(2));
                }
            }
            if (share_vertex.size() > 2) {
                std::cout << "share vertices number warning: " << share_vertex.size() << std::endl;
            }
        }

        double wij(int pi, int pj, int si, int sj = -1)
        {
            Eigen::Vector3d p1 = _origin_petal.col(pi);
            Eigen::Vector3d p2 = _origin_petal.col(pj);
            Eigen::Vector3d p3 = _origin_petal.col(si);

            double e12 = (p1 - p2).norm();
            double e13 = (p1 - p3).norm();
            double e23 = (p2 - p3).norm();
            double alpha_cos = fabs((e23*e23 + e13*e13 - e12*e12)/(2*e23*e13));

            double beta_cos = 0;
            if (sj != -1)
            {
                Eigen::Vector3d p4 = _origin_petal.col(sj);
                double e14 = (p1 - p4).norm();
                double e24 = (p2 - p4).norm();
                beta_cos = fabs((e14*e14+e24*e24-e12*e12)/(2*e14*e24));
            }

            return ((alpha_cos/sqrt(1-alpha_cos*alpha_cos))+(beta_cos/sqrt(1-beta_cos*beta_cos)))/2;
        }
    }DeformPetal;

    static std::vector<DeformPetal> deform_petals_;

    static int iter_num_;
    static double eps_;

    static double lambda_data_fitting_;
    static double lambda_skel_smooth_;
    static double noise_p_;

public:
    Solver(PointCloud* point_cloud, Flower* flower);

    void setFlower(Flower* flower);
    void setPointCloud(PointCloud* point_cloud);

    void deform();

    void global_deform();

protected:
    void deform(int petal_id);
    void deforming(int petal_id);
    void lbs(int petal_id);

    void e_step(int petal_id);
    double m_step(int petal_id);

    void initbuild(int petal_id);
    void left_sys(int petal_id);
    void right_sys(int petal_id);

    double solve(int petal_id);
    double energy(int petal_id);

    void projection(int petal_id);
    void update(int petal_id);

    double gaussian(int petal_id, int m_id, int c_id);


    void global_deforming();
    void lbs();

    void e_step();
    double m_step();

    void initbuild();
    void left_sys();
    void right_sys();

    double solve();
    double energy();

    void projection();
    void update();


protected:
    void init();
    void initParas();
    void initTerms();
    double zero_correction(double value);

private:
    Flower* flower_;
    PointCloud* point_cloud_;

    std::vector<DataFittingTerm> data_term_;
    std::vector<ARAPTerm> arap_term_;
    std::vector<SkelSmoothTerm> skel_term_;

    std::vector<std::vector<Eigen::MatrixXd>> A_; // x, y, z
    std::vector<Eigen::Matrix3Xd> b_;

    Eigen::SparseLU<Eigen::SparseMatrix<double> > lu_solver_;

    int petal_num_;
};

#endif