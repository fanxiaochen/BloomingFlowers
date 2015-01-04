
#include "point_cloud.h"
#include "flower.h"

#include "deform_model.h"

DeformModel::DeformModel()
    :iter_num_(50), 
    eps_(1e-3)
{

}

DeformModel::DeformModel(PointCloud* point_cloud, Flower* flower)
    :iter_num_(50), 
    eps_(1e-3),
    point_cloud_(point_cloud),
    flower_(flower)
{

}

DeformModel::~DeformModel()
{

}

void DeformModel::setIterNum(int iter_num)
{
    iter_num_ = iter_num;
}

void DeformModel::setEps(float eps)
{
    eps_ = eps;
}

void DeformModel::setPointCloud(PointCloud* point_cloud)
{
    point_cloud_ = point_cloud;
}

void DeformModel::setFlower(Flower* flower)
{
    flower_ = flower;
}

void DeformModel::deform()
{
    // initial step

    int iter_num = 0;

    do 
    {
        e_step();

        m_step();

    } while (iter_num < iter_num_);
}


void DeformModel::e_step()
{

}

void DeformModel::m_step()
{

}