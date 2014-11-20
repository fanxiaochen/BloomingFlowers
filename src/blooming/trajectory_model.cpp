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

}

value_type Trajectories::distance(const TrajectoryPath& path_1, const TrajectoryPath& path_2)
{
    value_type dist = 0;
    
    for (size_t i = 0, i_end = path_1._trajectory.size(); i < i_end; i ++)
    {
        for (size_t j = 0, j_end = path_2._trajectory.size(); j < j_end; j ++)
        {

        }
    }
}

