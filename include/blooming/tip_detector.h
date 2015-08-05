#ifndef TIP_DETECTOR_H
#define TIP_DETECTOR_H
#include <vector>
#include <pcl/kdtree/kdtree_flann.h>
#include "point_cloud.h"


// fake deleter
// since the same point cloud is referenced both by pcl smart pointer and 
// osg smart pointer, when tip has been detected, the pcl smart pointer will
// delete the point cloud while it's still referenced by osg for rendering.
struct NullDeleter {template<typename T> void operator()(T*) {} };

// pp of type some_t defined somewhere
//boost::shared_ptr<some_t> x(pp, NullDeleter() );

class TipDetector
{

public:
    TipDetector();
    TipDetector(PointCloud* point_cloud);
    ~TipDetector();

    void setPointCloud(PointCloud* point_cloud);

    void detectBoundary(int bin_number = 12, float knn_radius = 5, float boundary_limit = 0.2);

    void detectTips(int bin_number = 12, float knn_radius = 5, float boundary_limit = 0.2, float corner_limit = 0.3);

private:
    // based on boundary
    void detectCorner(float corner_limit);

    bool boundary(int index);
    bool corner(int index);

    pcl::IndicesPtr knn(int index, PointCloud::Ptr point_cloud); // index based on point cloud

private:
    boost::shared_ptr<PointCloud>   point_cloud_;

    int                             bin_number_;
    float                           interval_;
    float                           radius_;

    std::vector<int>                boundary_indices_;
    std::vector<Eigen::Matrix3f>    boundary_evs_;
    boost::shared_ptr<PointCloud>   boundary_cloud_;

    float                           boundary_limit_;
    float                           corner_limit_;

    pcl::KdTreeFLANN<Point>         kdtree_;

};
#endif