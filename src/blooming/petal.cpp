
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

    /*for (size_t i = 0, i_end = petal.getVertices()->size(); i < i_end; ++ i)
    {
    this->vertices_[i] = petal.getVertices()[i];
    }

    for (size_t i = 0, i_end = petal.getFaces().size(); i < i_end; ++ i)
    {
    this->faces_[i] = petal.getFaces()[i];
    }

    for (size_t i = 0, i_end = petal.getAdjList().size(); i < i_end; ++ i)
    {
    this->adj_list_[i] = petal.getAdjList()[i];
    }
    */
    
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