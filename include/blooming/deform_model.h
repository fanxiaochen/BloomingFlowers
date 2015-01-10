#ifndef DEFORM_MODEL_H
#define DEFORM_MODEL_H

#include <iostream>

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

class PointCloud;
class Flower;

class DeformModel
{
private:
    typedef Eigen::MatrixXd CorresMatrix;
    typedef Eigen::Matrix3Xd CloudMatrix;
    typedef Eigen::Matrix3Xd PetalMatrix;
    typedef Eigen::Matrix3Xd CovMatrix;
    typedef std::vector<int> VisList;
    typedef std::vector<std::vector<int> > AdjList;
    typedef std::vector<Eigen::Vector3i > FaceList;
    typedef Eigen::SparseMatrix<double> WeightMatrix;
    typedef std::vector<Eigen::Matrix3d> RotList;
    typedef std::vector<Eigen::Vector3d> HardCtrsPos;
    typedef std::vector<int> HardCtrsIdx;

    struct DeformPetal
    {
        PetalMatrix     _origin_petal;
        PetalMatrix     _petal_matrix;
        CloudMatrix     _cloud_matrix;
        CorresMatrix    _corres_matrix;
        CovMatrix       _cov_matrix;
        VisList         _vis_list;
        AdjList         _adj_list;
        FaceList        _face_list;
        WeightMatrix    _weight_matrix;
        RotList         _R_list;
        HardCtrsPos     _hc_pos;
        HardCtrsIdx     _hc_idx;

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

        int isHardCtrs(int id)
        {
            // binary search
            int k = 0, k_end = _hc_idx.size();

            if (k_end == 0)
                return -1;

            while (k <= k_end)
            {
                int m = (k + k_end) / 2;

                if (id == _hc_idx[m])
                    return m;
                else if (id < _hc_idx[m])
                    k_end = m - 1;
                else 
                    k = m + 1;
            }

            return -1;
        }
    };

public:
    DeformModel();
    DeformModel(PointCloud* point_cloud, Flower* flower);
    virtual ~DeformModel();

    void setPointCloud(PointCloud* point_cloud);
    void setFlower(Flower* flower);

    //void setHardCtrs(HardCtrsPos hc_pos, HardCtrsIdx hc_idx);

    void setIterNum(int iter_num);
    void setEps(double eps);
    void setLambda(double lambda);
    
    inline PointCloud* getPointCloud(){ return point_cloud_; }
    inline Flower* getFlower(){ return flower_; }

    void deform();

protected:
    void e_step();
    double m_step();

    void e_step(int petal_id);

    void visibility();
    void initialize();

    double gaussian(int petal_id, int m_id, int c_id);

    void buildWeightMatrix(int petal_id);

    void updateLeftSys();
    void updateRightSys();
    double solve();

    void updateLeftSys(int petal_id);
    void updateRightSys(int petal_id);

    void updateRotation(int petal_id);
    void updateRotation();
    void initRotation();

    double energy();
    double zero_correction(double value);

    void deforming();

private:
    PointCloud* point_cloud_;
    Flower* flower_;

    std::vector<DeformPetal> deform_petals_;

    std::vector<Eigen::SparseMatrix<double> > L_;
    Eigen::Matrix3Xd d_;
    Eigen::SparseLU<Eigen::SparseMatrix<double> > lu_solver_;
//    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double> > lu_solver_;

    int petal_num_;

    int iter_num_;
    double eps_;

    double lambda_;
    double noise_p_;
};
#endif