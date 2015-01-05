#ifndef DEFORM_MODEL_H
#define DEFORM_MODEL_H

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
    typedef std::vector<std::vector<int> > FaceList;

    struct DeformPetal
    {
        PetalMatrix     _petal_matrix;
        CloudMatrix     _cloud_matrix;
        CorresMatrix    _corres_matrix;
        CovMatrix       _cov_matrix;
        VisList         _vis_list;
        AdjList         _adj_list;
        FaceList        _face_list;
    };

public:
    DeformModel();
    DeformModel(PointCloud* point_cloud, Flower* flower);
    virtual ~DeformModel();

    void setPointCloud(PointCloud* point_cloud);
    void setFlower(Flower* flower);

    void setIterNum(int iter_num);
    void setEps(float eps);
    
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

    void buildWeightMatrix();

private:
    PointCloud* point_cloud_;
    Flower* flower_;

    std::vector<DeformPetal> deform_petals_;

    /*std::vector<CloudMatrix> cloud_mats_;
    std::vector<PetalMatrix> petal_mats_;

    std::vector<CorresMatrix> corres_mats_;
    std::vector<CovMatrix> cov_mats_;
    std::vector<VisList> vis_lists_;

    std::vector<AdjList> adj_lists_;
    std::vector<FaceList> face_lists_;*/

    int petal_num_;

    int iter_num_;
    int eps_;

    float noise_p_;
};
#endif