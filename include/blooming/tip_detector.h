#ifndef TIP_DETECTOR_H
#define TIP_DETECTOR_H
#include <vector>

#include "point_cloud.h"

class TipDetector
{
public:
    TipDetector(PointCloud* point_cloud);

    void setPointCloud(PointCloud* point_cloud);
    //void setTrajectoryModel(TrajectoryModel* trajectory_model);

    void detect(int bin_number, float radius);

private:
    void detectBoundary();
    void detectCorner();

   // bool tip(int index);
    bool boundary(int index);
    bool corner(int index);

    std::vector<int> knn(int index, PointCloud* point_cloud); // index based on point cloud

private:
    PointCloud*         point_cloud_;
    int                 bin_number_;
    float               interval_;
    std::vector<int>    bin_count_;
    float               radius_;

    std::vector<int>    boundary_indices_;
    osg::ref_ptr<PointCloud> boundary_cloud_;

    float               boundary_limit_;
    float               corner_limit_;
    //TrajectoryModel*		trajectory_model_;

};
#endif