
#include <pcl/common/pca.h>

#include "main_window.h"
#include "point_cloud.h"
#include "tip_detector.h"

TipDetector::TipDetector()
    :boundary_limit_(0.2),
    corner_limit_(0.2),
    radius_(5),
    bin_number_(12),
    boundary_cloud_(new PointCloud)
{

}

TipDetector::TipDetector(PointCloud* point_cloud)
    :point_cloud_(point_cloud),
    boundary_limit_(0.2),
    corner_limit_(0.2),
    radius_(5),
    bin_number_(12),
    boundary_cloud_(new PointCloud)
{
    type_ = POINTCLOUD_DETECTOR;
}

TipDetector::TipDetector(Flower* flower)
    :flower_(flower),
    boundary_limit_(0.2),
    corner_limit_(0.2),
    radius_(5),
    bin_number_(12),
    boundary_cloud_(new PointCloud)
{
    type_ = FLOWER_DETECTOR;
}

TipDetector::~TipDetector()
{
}

void TipDetector::setPointCloud(PointCloud* point_cloud)
{
    point_cloud_ = boost::shared_ptr<PointCloud>(point_cloud, NullDeleter());
    type_ = POINTCLOUD_DETECTOR;
}

void TipDetector::setFlower(Flower* flower)
{
    flower_ = boost::shared_ptr<Flower>(flower, NullDeleter());
    type_ = FLOWER_DETECTOR;
}

void TipDetector::detectTips(int bin_number, float knn_radius, float boundary_limit, float corner_limit)
{
    //// build point cloud for flower
    //if (type_ == FLOWER_DETECTOR)
    //    buildCloudIndex();

    // detect boundary points on point cloud / flower
    detectBoundary(bin_number, knn_radius, boundary_limit);

    // detect corners on boundary points
    detectCorner(corner_limit);

    //// recover flower boundary/corner index
    //if (type_ == FLOWER_DETECTOR)
    //    recoverFlowerIndex();
}

void TipDetector::detectBoundary(int bin_number, float knn_radius, float boundary_limit)
{
    // build point cloud for flower
    if (type_ == FLOWER_DETECTOR)
        buildCloudIndex();

    bin_number_ = bin_number;
    radius_ = knn_radius;
    boundary_limit_ = boundary_limit;
    interval_ = 360.0 / bin_number_;

    boundary_indices_.clear();
    boundary_evs_.clear();

    kdtree_.setInputCloud (point_cloud_);

    for (size_t i = 0, i_end = point_cloud_->size(); i < i_end; ++ i)
    {
        if (boundary(i)) 
            boundary_indices_.push_back(i);
    }

    point_cloud_->getBoundary() = boundary_indices_;

    //build boundary point cloud
     // clear() function is not unique
    boundary_cloud_->pcl::PointCloud<Point>::clear();
    for (int index : boundary_indices_)
    {
        boundary_cloud_->push_back(point_cloud_->at(index));
    }

    if (type_ == FLOWER_DETECTOR)
        recoverFlowerBoundaryIndex();
}

void TipDetector::detectCorner(float corner_limit)
{
    corner_limit_ = corner_limit;

    point_cloud_->getTips().clear();

    kdtree_.setInputCloud (boundary_cloud_);

    for (size_t i = 0, i_end = boundary_indices_.size(); i < i_end; ++ i)
    {
        // based on boundary, if corner detected, here's the tip
        if (corner(i))
            point_cloud_->getTips().push_back(boundary_indices_[i]);
    }

    if (type_ == FLOWER_DETECTOR)
        recoverFlowerTipIndex();
}

// index based on point cloud
bool TipDetector::boundary(int index)
{
    // pca
    pcl::PCA<Point> pca;
    pcl::PointCloud<Point>::Ptr pcp(point_cloud_);
    pcl::IndicesPtr knn_idx = knn(index, point_cloud_);
    if (knn_idx->size() <= 2) return false; // at least three points for PCA

    pca.setInputCloud(pcp);
    pca.setIndices(knn_idx);
    Eigen::Vector3f values = pca.getEigenValues();
    Eigen::Matrix3f evs = pca.getEigenVectors();

    boundary_evs_.push_back(evs); // store eigen vectors for each point

    // projection
    Eigen::Vector3f xvec(evs(0, 0), evs(1, 0), evs(2, 0));
    Eigen::Vector3f yvec(evs(0, 1), evs(1, 1), evs(2, 1));

    Point& c = point_cloud_->at(index);
    Eigen::Vector3f center(c.x, c.y, c.z);
    std::vector<int> bin_count = std::vector<int>(bin_number_, 0);
    for (int i : *knn_idx)
    {
        Point& p = point_cloud_->at(i);
        if (p == c) continue;

        Eigen::Vector3f point(p.x, p.y, p.z);
        float xlen = xvec.dot(point-center) / xvec.norm();
        float ylen = yvec.dot(point-center) / yvec.norm();
        float angle = atan2 (ylen, xlen) * 180 / M_PI;
        int bin_idx = int((angle+180) / interval_) % bin_number_;
        bin_count[bin_idx] ++;
    }

    // bin count
    int zero_cnt = 0;
    for (int count : bin_count)
    {
        if (count == 0)
            zero_cnt ++;
    }

    if (float(zero_cnt)/bin_number_ > boundary_limit_)
        return true;
    else return false;
}

// index based on boundary points
bool TipDetector::corner(int index)
{
    // projection into original plane
    PointCloud::Ptr plane_points(new PointCloud);

    Eigen::Matrix3f evs = boundary_evs_[boundary_indices_[index]];
    Eigen::Vector3f xvec(evs(0, 0), evs(1, 0), evs(2, 0));
    Eigen::Vector3f yvec(evs(0, 1), evs(1, 1), evs(2, 1));

    Point& c = boundary_cloud_->at(index);
    Eigen::Vector3f center(c.x, c.y, c.z);
    pcl::IndicesPtr knn_idx = knn(index, boundary_cloud_);
    if (knn_idx->size() <= 2) return false; // at least three points for PCA
    for (int i : *knn_idx)
    {
        Point& p = boundary_cloud_->at(i);
        Eigen::Vector3f point(p.x, p.y, p.z);
        float xlen = xvec.dot(point-center) / xvec.norm();
        float ylen = yvec.dot(point-center) / yvec.norm();
        plane_points->push_back([](float x, float y, float z){
            Point point;
            point.x = x;
            point.y = y;
            point.z = z;
            return point;
        }(xlen, ylen, 0));
    }
    
    // pca
    pcl::PCA<Point> pca;
    pcl::PointCloud<Point>::Ptr pcp(plane_points);
    pca.setInputCloud(pcp);
    pca.setIndices([](int n){
        pcl::IndicesPtr indices(new std::vector<int>);
        for (int i = 0; i < n; ++ i)
            indices->push_back(i);
        return indices;
    }(knn_idx->size()));

    Eigen::Vector3f values = pca.getEigenValues();

    float min_lambda = values(1);
    float max_lambda = values(0);
    //std::cout << min_lambda / max_lambda << std::endl;
    if (min_lambda / max_lambda > corner_limit_)
        return true;
    else return false;
}

pcl::IndicesPtr TipDetector::knn(int index, PointCloud::Ptr point_cloud)
{
    // Neighbors within radius search
    std::vector<int>* pointIdxRadiusSearch = new std::vector<int>;
    std::vector<float> pointRadiusSquaredDistance;

    pcl::IndicesPtr knn_idx(pointIdxRadiusSearch);

    const Point& point = point_cloud->at(index);
    kdtree_.radiusSearch (point, radius_, *pointIdxRadiusSearch, pointRadiusSquaredDistance);

    return knn_idx;
}

void TipDetector::buildPetalMap()
{
    petal_map_.push_back(0);
    int num = 0;
    Petals& petals = flower_->getPetals();
    for (Petal& petal : petals)
    {
        num += petal.getVertices()->size();
        petal_map_.push_back(num);
    }
}

void TipDetector::recoverPetalIndex(int cloud_index, int& petal_id, int& petal_index)
{
    for (int i = 0, i_end = petal_map_.size()-1; i < i_end; ++ i)
    {
        if (cloud_index >= petal_map_[i] && cloud_index < petal_map_[i+1])
        {
            petal_id = i;
            petal_index = cloud_index - petal_map_[i];
            return;
        }
    }

    petal_id = -1;
    petal_index = -1;
}

bool TipDetector::onEdge(int petal_id, int petal_index)
{
    Petals& petals = flower_->getPetals();
    Petal& petal = petals[petal_id];
    std::vector<int>& edge_idx = petal.getEdgeIndex();
    for (int index : edge_idx)
    {
        if (petal_index == index)
            return true;
    }
    return false;
}

void TipDetector::buildCloudIndex()
{
    point_cloud_ = boost::shared_ptr<PointCloud>(new PointCloud);

    Petals& petals = flower_->getPetals();
    for (Petal& petal : petals)
    {
        osg::ref_ptr<osg::Vec3Array> vertices = petal.getVertices();
        for (size_t i = 0, i_end = vertices->size(); i < i_end; ++ i)
        {
            osg::Vec3& vertice = vertices->at(i);
            Point point;
            point.x = vertice.x();
            point.y = vertice.y();
            point.z = vertice.z();
            point_cloud_->push_back(point);
        }
    }

    buildPetalMap();
}

void TipDetector::recoverFlowerBoundaryIndex()
{
    std::vector<int>& boundary_index = point_cloud_->getBoundary();
    for (int index : boundary_index)
    {
        int petal_id, petal_index;
        recoverPetalIndex(index, petal_id, petal_index);
        if (onEdge(petal_id, petal_index))
        {
            Petal& petal = flower_->getPetals().at(petal_id);
            petal.getDetectedBoundary().push_back(petal_index);
        }
    }
}

void TipDetector::recoverFlowerTipIndex()
{
    std::vector<int>& tip_index = point_cloud_->getTips();
    for (int index : tip_index)
    {
        int petal_id, petal_index;
        recoverPetalIndex(index, petal_id, petal_index);
        if (onEdge(petal_id, petal_index))
        {
            Petal& petal = flower_->getPetals().at(petal_id);
            petal.getDetectedTips().push_back(petal_index);
        }
    }
}