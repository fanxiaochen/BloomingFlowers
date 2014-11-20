#include "points_file_system.h"
#include "trajectory_model.h"

Trajectories::Trajectories(PointsFileSystem* points_file_system)
{
    points_file_system_ = points_file_system;
}

Trajectories::~Trajectories()
{

}

void Trajectories::load(const std::string& file)
{

}

void Trajectories::save(const std::string& file)
{

}

void Trajectories::build()
{

}

void Trajectories::clustering()
{
    k_means();
    return;
}

value_type Trajectories::distance(const TrajectoryPath& path_1, const TrajectoryPath& path_2)
{
    int path1_size = path_1._trajectory.size();
    int path2_size = path_2._trajectory.size();
    assert(path1_size == path2_size);


    value_type dist = 0;
    int n = path1_size;
    
    for (size_t i = 0, i_end = n; i < i_end; i ++)
    {
        int idx_1 = path_1._trajectory.at(i);
        int idx_2 = path_2._trajectory.at(i);

        PointCloud* point_cloud = points_file_system_->getPointCloud(i);
        const Point& point_1 = point_cloud->at(idx_1);
        const Point& point_2 = point_cloud->at(idx_2);

        dist += sqrt(pow(point_1.x-point_2.x,2)+pow(point_1.y-point_2.y,2)+pow(point_1.z-point_2.z,2));
    }

    return dist / n;
}

void Trajectories::k_means()
{

}

