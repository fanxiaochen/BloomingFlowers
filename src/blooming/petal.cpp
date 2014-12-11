
#include <pcl/kdtree/kdtree_flann.h>

#include "petal.h"

Petal::Petal(int id)
    :petal_id_(id)
{

}

Petal::Petal(Petal& petal)
{
    this->petal_id_ = petal.petal_id_;

    this->deform_model_ = petal.getDeformModel();

    *(this->getVertices()) = *(petal.getVertices());

    this->getFaces() = petal.getFaces();

    this->getAdjList() = petal.getAdjList();

    for (size_t i = 0, i_end = petal.size(); i < i_end; ++ i)
    {
        this->at(i) = petal.at(i);
    }
    
}

Petal::~Petal()
{

}

void Petal::setMeshModel(MeshModel* mesh_model)
{
    vertices_ = mesh_model->getVertices();
    faces_ = mesh_model->getFaces();
    adj_list_ = mesh_model->getAdjList();
    deform_model_ = mesh_model->getDeformModel();
}

void Petal::setPointCloud(PointCloud* point_cloud)
{
    for (size_t i = 0, i_end = point_cloud->size(); i < i_end; ++ i)
    {
        this->at(i) = point_cloud->at(i);
    }
}

void Petal::updateImpl()
{
    visualizePetal();
    return;
}

void Petal::visualizePetal()
{
    visualizeMesh();
    return;
}

void Petal::searchNearestIdx(Petal& petal, std::vector<int>& idx)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0, i_end = this->getVertices()->size(); i < i_end; ++ i)
    {
        osg::Vec3& point = this->getVertices()->at(i);

        cloud->points[i].x = point.x();
        cloud->points[i].y = point.y();
        cloud->points[i].z = point.z();
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud (cloud);

    int K = 1;

    // K nearest neighbor search

    for (size_t i = 0, i_end = petal.getVertices()->size(); i < i_end; ++ i)
    {
        pcl::PointXYZ searchPoint;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        osg::Vec3& point = petal.getVertices()->at(i);

        searchPoint.x = point.x();
        searchPoint.y = point.y();
        searchPoint.z = point.z();

        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            idx.push_back(pointIdxNKNSearch[0]);
    }
}
