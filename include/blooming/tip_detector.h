#ifndef TIP_DETECTOR_H
#define TIP_DETECTOR_H
#include <vector>

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


    void detect(int bin_number, float radius);

private:
    void detectBoundary();
    void detectCorner();

   // bool tip(int index);
    bool boundary(int index);
    bool corner(int index);

    pcl::IndicesPtr knn(int index, PointCloud::Ptr point_cloud); // index based on point cloud

private:
    boost::shared_ptr<PointCloud>   point_cloud_;
    int                             bin_number_;
    float                           interval_;
    //std::vector<int>                bin_count_;
    float                           radius_;

    std::vector<int>                boundary_indices_;
    boost::shared_ptr<PointCloud>   boundary_cloud_;

    float                           boundary_limit_;
    float                           corner_limit_;
    //TrajectoryModel*		trajectory_model_;

};
#endif