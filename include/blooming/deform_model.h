#ifndef DEFORM_MODEL_H
#define DEFORM_MODEL_H

class PointCloud;
class Flower;

class DeformModel
{
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

private:
    PointCloud* point_cloud_;
    Flower* flower_;

    int iter_num_;
    int eps_;


};
#endif