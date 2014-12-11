
#include "petal.h"

Petal::Petal(int id)
    :petal_id_(id)
{

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

}