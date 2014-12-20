//
//#include <pcl/kdtree/kdtree_flann.h>
//
//#include "main_window.h"
//#include "petal.h"
//
//Petal::Petal(int id)
//    :petal_id_(id),
//    point_cloud_(NULL),
//    mesh_model_(NULL)
//{
//
//}
//
//Petal::Petal(int id, MeshModel* mesh_model)
//    :petal_id_(id),
//    mesh_model_(mesh_model)
//{
//
//}
//
//Petal::Petal(int id, PointCloud* point_cloud, MeshModel* mesh_model)
//    :petal_id_(id),
//    point_cloud_(point_cloud),
//    mesh_model_(mesh_model)
//{
//
//}
//
////Petal::Petal(Petal& petal)
////{
////    this->petal_id_ = petal.petal_id_;
////
////    this->deform_model_ = petal.getDeformModel();
////
////    *(this->getVertices()) = *(petal.getVertices());
////
////    this->getFaces() = petal.getFaces();
////
////    this->getAdjList() = petal.getAdjList();
////
////    for (size_t i = 0, i_end = petal.size(); i < i_end; ++ i)
////    {
////        this->at(i) = petal.at(i);
////    }
////    
////}
//
//Petal::~Petal()
//{
//
//}
//
//void Petal::setMeshModel(MeshModel* mesh_model)
//{
//    mesh_model_ = mesh_model;
//}
//
//void Petal::setPointCloud(PointCloud* point_cloud)
//{
//    point_cloud_ = point_cloud;
//}
//
//void Petal::showPointCloud()
//{
//    MainWindow::getInstance()->getSceneWidget()->addSceneChild(point_cloud_);
//}
//
//void Petal::showMeshModel()
//{
//    MainWindow::getInstance()->getSceneWidget()->addSceneChild(mesh_model_);
//}
//
//
//
//void Petal::updateImpl()
//{
//    visualizePetal();
//    return;
//}
//
//void Petal::visualizePetal()
//{
//    visualizeMesh();
//    return;
//}
//
//void Petal::searchNearestIdx(Petal& petal, std::vector<int>& idx)
//{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//
//    for (size_t i = 0, i_end = this->getVertices()->size(); i < i_end; ++ i)
//    {
//        osg::Vec3& point = this->getVertices()->at(i);
//
//        pcl::PointXYZ pcl_point(point.x(), point.y(), point.z());
//        cloud->push_back(pcl_point);
//    }
//
//    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//
//    kdtree.setInputCloud (cloud);
//
//    int K = 1;
//
//    // K nearest neighbor search
//
//    for (size_t i = 0, i_end = petal.getVertices()->size(); i < i_end; ++ i)
//    {
//        pcl::PointXYZ searchPoint;
//        std::vector<int> pointIdxNKNSearch(K);
//        std::vector<float> pointNKNSquaredDistance(K);
//
//        osg::Vec3& point = petal.getVertices()->at(i);
//
//        searchPoint.x = point.x();
//        searchPoint.y = point.y();
//        searchPoint.z = point.z();
//
//        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
//            idx.push_back(pointIdxNKNSearch[0]);
//    }
//}
//
//void Petal::searchNearestIdx(PointCloud& point_cloud, osg::Vec3Array& knn_pos)
//{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//
//    for (size_t i = 0, i_end = point_cloud.size(); i < i_end; ++ i)
//    {
//        const Point& point = point_cloud.at(i);
//
//        pcl::PointXYZ pcl_point(point.x, point.y, point.z);
//        cloud->push_back(pcl_point);
//    }
//
//    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//
//    kdtree.setInputCloud (cloud);
//
//    int K = 1;
//
//    // K nearest neighbor search
//
//    for (size_t i = 0, i_end = this->getVertices()->size(); i < i_end; ++ i)
//    {
//        pcl::PointXYZ searchPoint;
//        std::vector<int> pointIdxNKNSearch(K);
//        std::vector<float> pointNKNSquaredDistance(K);
//
//        osg::Vec3& point = this->getVertices()->at(i);
//
//        searchPoint.x = point.x();
//        searchPoint.y = point.y();
//        searchPoint.z = point.z();
//
//        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
//        {
//            osg::Vec3 pos(cloud->at(pointIdxNKNSearch[0]).x, cloud->at(pointIdxNKNSearch[0]).y, cloud->at(pointIdxNKNSearch[0]).z);
//            knn_pos.push_back(pos);
//        }
//    }
//}
