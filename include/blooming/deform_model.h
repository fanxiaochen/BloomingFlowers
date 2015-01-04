#ifndef DEFORM_MODEL_H
#define DEFORM_MODEL_H

#include <vector>

#include <Eigen/Dense>

class PointCloud;
class Flower;

class DeformModel
{
private:
    typedef Eigen::MatrixXf CorresMatrix;
    typedef Eigen::Matrix3Xf CloudMatrix;
    typedef Eigen::Matrix3Xf PetalMatrix;
    typedef Eigen::Vector3f CovMatrix;
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

    void visibility();
    void initialize();

    float gaussian();

private:
    PointCloud* point_cloud_;
    Flower* flower_;

    std::vector<CloudMatrix> cloud_mats_;
    std::vector<PetalMatrix> petal_mats_;

    std::vector<CorresMatrix> corres_mats_;
    std::vector<CovMatrix> cov_mats_;

    int petal_num_;

    int iter_num_;
    int eps_;
};
#endif