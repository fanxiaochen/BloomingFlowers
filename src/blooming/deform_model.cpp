
#include "point_cloud.h"
#include "flower.h"

#include "deform_model.h"

DeformModel::DeformModel()
    :petal_num_(0),
    iter_num_(50), 
    eps_(1e-3)
{

}

DeformModel::DeformModel(PointCloud* point_cloud, Flower* flower)
    :petal_num_(0),
    iter_num_(50), 
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

void DeformModel::visibility()
{
    // now just using knn to determine visibility
}


void DeformModel::initialize()
{
    // init petal matrix
    Petals& petals = flower_->getPetals();
    petal_num_ = petals.size();

    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        Petal& petal = petals.at(i);

        std::vector<int> knn_idx;
        petal.searchNearestIdx(point_cloud_, knn_idx);
        
        int petal_size = petal.getVertices()->size();
        PetalMatrix pm(3, petal_size);

        for (size_t j = 0, j_end = petal_size; j < j_end; ++ j)
        {
            pm.col(j) << point_cloud_->at(knn_idx[j]).x, point_cloud_->at(knn_idx[j]).y, point_cloud_->at(knn_idx[j]).z;
        }
        petal_mats_.push_back(pm);
    }

    // init cloud matrix
    point_cloud_->flower_segmentation(flower_); // using knn to segment the point cloud to petals

    for (size_t i = 0, i_end = petal_num_; i < i_end; ++ i)
    {
        osg::ref_ptr<PointCloud> petal_cloud = point_cloud_->getPetalCloud(i);
        CloudMatrix cm(3, petal_cloud->size());
        if (petal_cloud != NULL)
        {
            for (size_t j = 0, j_end = petal_cloud->size(); j < j_end; ++ j)
            {
                cm.col(j) << petal_cloud->at(j).x, petal_cloud->at(j).y, petal_cloud->at(j).z;
            }
        }
        cloud_mats_.push_back(cm);
    }

    // init covariance matrix 

}

float DeformModel::gaussian()
{

}