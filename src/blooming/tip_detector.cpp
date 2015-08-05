
#include <pcl/common/pca.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "point_cloud.h"
#include "tip_detector.h"

TipDetector::TipDetector()
    :boundary_limit_(0.2),
    corner_limit_(0.2),
    boundary_cloud_(new PointCloud)
{

}

TipDetector::TipDetector(PointCloud* point_cloud)
    :point_cloud_(point_cloud),
    boundary_limit_(0.2),
    corner_limit_(0.2),
    boundary_cloud_(new PointCloud)
{

}

TipDetector::~TipDetector()
{
}

void TipDetector::setPointCloud(PointCloud* point_cloud)
{
    point_cloud_ = boost::shared_ptr<PointCloud>(point_cloud, NullDeleter());
}

void TipDetector::detect(int bin_number, float radius)
{
    bin_number_ = bin_number;
    radius_ = radius;

    interval_ = 360.0 / bin_number_;
    //bin_count_ = std::vector<int>(bin_number_, 0);

    // detect boundary points on point cloud
    detectBoundary();

    // detect corners on boundary points
    detectCorner();

}

void TipDetector::detectBoundary()
{
    boundary_indices_.clear();

    for (size_t i = 0, i_end = point_cloud_->size(); i < i_end; ++ i)
    {
        if (boundary(i)) 
            boundary_indices_.push_back(i);
    }
    /*boundary(0);
    boundary_indices_.push_back(0);*/
    //build boundary point cloud

     // clear() function is not unique
    boundary_cloud_->pcl::PointCloud<Point>::clear();
    for (int index : boundary_indices_)
    {
        boundary_cloud_->push_back(point_cloud_->at(index));
    }
}

void TipDetector::detectCorner()
{
    point_cloud_->getTips().clear();

    for (size_t i = 0, i_end = boundary_indices_.size(); i < i_end; ++ i)
    {
        // if corner, it's tip
        if (corner(i))
            point_cloud_->getTips().push_back(boundary_indices_[i]);
    }
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
        int bin_idx = (angle+180) / interval_;
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
    // pca
    pcl::PCA<Point> pca;
    pcl::PointCloud<Point>::Ptr pcp(boundary_cloud_);
    pcl::IndicesPtr knn_idx = knn(index, boundary_cloud_);
    if (knn_idx->size() <= 2) return false; // at least three points for PCA

    pca.setInputCloud(pcp);
    pca.setIndices(knn_idx);
    Eigen::Vector3f values = pca.getEigenValues();
    Eigen::Matrix3f evs = pca.getEigenVectors();

    float min_lambda = values(1);
    float max_lambda = values(0);
    std::cout << min_lambda / max_lambda << std::endl;
    if (min_lambda / max_lambda > corner_limit_)
        return true;
    else return false;
}

pcl::IndicesPtr TipDetector::knn(int index, PointCloud::Ptr point_cloud)
{
    //pcl::PointCloud<Point>::Ptr cloud (point_cloud);

    pcl::KdTreeFLANN<Point> kdtree;
    kdtree.setInputCloud (point_cloud);

    // Neighbors within radius search
    std::vector<int>* pointIdxRadiusSearch = new std::vector<int>;
    std::vector<float> pointRadiusSquaredDistance;

    pcl::IndicesPtr knn_idx(pointIdxRadiusSearch);

    const Point& point = point_cloud_->at(index);
    kdtree.radiusSearch (point, radius_, *pointIdxRadiusSearch, pointRadiusSquaredDistance);

    return knn_idx;
}
