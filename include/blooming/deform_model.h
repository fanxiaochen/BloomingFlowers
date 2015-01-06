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
    typedef Eigen::MatrixXf CorresMatrix;
    typedef Eigen::Matrix3Xf CloudMatrix;
    typedef Eigen::Matrix3Xf PetalMatrix;
    typedef Eigen::Vector3f CovMatrix;
    typedef std::vector<int> VisList;
    typedef std::vector<std::vector<int> > AdjList;
    typedef std::vector<Eigen::Vector3i > FaceList;
    typedef Eigen::SparseMatrix<float> WeightMatrix;
    typedef std::vector<Eigen::Matrix3f> RotList;

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

        float wij(int pi, int pj, int si, int sj = -1)
        {
            Eigen::Vector3f p1 = _petal_matrix.col(pi);
            Eigen::Vector3f p2 = _petal_matrix.col(pj);
            Eigen::Vector3f p3 = _petal_matrix.col(si);

            float e1 = sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1])+(p1[2]-p2[2])*(p1[2]-p2[2]));
            float e2 = sqrt((p1[0]-p3[0])*(p1[0]-p3[0])+(p1[1]-p3[1])*(p1[1]-p3[1])+(p1[2]-p3[2])*(p1[2]-p3[2]));
            float e3 = sqrt((p3[0]-p2[0])*(p3[0]-p2[0])+(p3[1]-p2[1])*(p3[1]-p2[1])+(p3[2]-p2[2])*(p3[2]-p2[2]));
            float alpha_cos = (e3*e3+e2*e2-e1*e1)/(2*e3*e2);
            float beta_cos = 0;
            if (sj != -1) {
                Eigen::Vector3f p4 = _petal_matrix.col(sj);
                float e4 = sqrt((p1[0]-p4[0])*(p1[0]-p4[0])+(p1[1]-p4[1])*(p1[1]-p4[1])+(p1[2]-p4[2])*(p1[2]-p4[2]));
                float e5 = sqrt((p4[0]-p2[0])*(p4[0]-p2[0])+(p4[1]-p2[1])*(p4[1]-p2[1])+(p4[2]-p2[2])*(p4[2]-p2[2]));
                beta_cos = (e4*e4+e5*e5-e1*e1)/(2*e4*e5);
            }
            return ((alpha_cos/sqrt(1-alpha_cos*alpha_cos))+(beta_cos/sqrt(1-beta_cos*beta_cos)))/2;
        }
    };

public:
    DeformModel();
    DeformModel(PointCloud* point_cloud, Flower* flower);
    virtual ~DeformModel();

    void setPointCloud(PointCloud* point_cloud);
    void setFlower(Flower* flower);

    void setIterNum(int iter_num);
    void setEps(float eps);
    void setLambda(float lambda);
    
    inline PointCloud* getPointCloud(){ return point_cloud_; }
    inline Flower* getFlower(){ return flower_; }

    void deform();

protected:
    void e_step();
    void m_step();

    void e_step(int petal_id);

    void visibility();
    void initialize();

    float gaussian(int petal_id, int m_id, int c_id);
    void covariance(const CloudMatrix& cloud_mat, CovMatrix& cov_mat);

    void buildWeightMatrix(int petal_id);

    void left_system();
    void right_system();
    void solve();

    void left_system(int petal_id);
    void right_system(int petal_id);

private:
    PointCloud* point_cloud_;
    Flower* flower_;

    std::vector<DeformPetal> deform_petals_;

    std::vector<Eigen::SparseMatrix<float> > L_;
    Eigen::Matrix3Xf d_;
    Eigen::SparseLU<Eigen::SparseMatrix<float>> lu_solver_;

    int petal_num_;

    int iter_num_;
    int eps_;

    float lambda_;
    float noise_p_;
};
#endif