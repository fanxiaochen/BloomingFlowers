
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
    TrajectoryPoint point_1, point_2;
    getPointsFromPath(path_1._id, point_1);
    getPointsFromPath(path_2._id, point_2);

    return distance(point_1, point_2);
}

value_type Trajectories::distance(const TrajectoryPoint& point_1, const TrajectoryPoint& point_2)
{
    int point1_size = point_1.size();
    int point2_size = point_2.size();
    assert(point1_size == point2_size);

    value_type dist = 0;
    int n = point1_size;

    for (size_t i = 0, i_end = n; i < i_end; i ++)
    {
        const Point& pt1 = point_1.at(i);
        const Point& pt2 = point_2.at(i);

        dist += sqrt(pow(pt1.x-pt2.x,2)+pow(pt1.y-pt2.y,2)+pow(pt1.z-pt2.z,2));
    }

    return dist;
}

void Trajectories::k_means()
{
    if (center_trajs_.size() != cluster_num_)
    {
        std::cerr << "the number of centers is not the same as cluster number..." << std::endl;
        return;
    }

    TrajectoryPoints next_centers;
    
    do 
    {
        if (!next_centers.empty())
        {
            center_trajs_ = next_centers;
            next_centers.clear();
        }

        size_t path_num = traj_paths_.size();
        for (size_t i = 0; i < path_num; i ++)
        {
            int cluster_id = determineCluster(traj_paths_[i]);
            traj_paths_[i]._label = cluster_id;
        }

        for (size_t i = 0; i < cluster_num_; i ++)
        {
            std::vector<int> ids;
            for (size_t j = 0; j < path_num; j ++)
            {
                if (traj_paths_[j]._label == i)
                    ids.push_back(j);
            }

            mean_path(ids, next_centers[i]);
        }

    } while (!terminal(center_trajs_, next_centers));

}

int Trajectories::determineCluster(const TrajectoryPath& path)
{
    TrajectoryPoint traj_point;
    getPointsFromPath(path._id, traj_point);

    value_type min = std::numeric_limits<value_type>::max();
    int cluster_id = -1;

    for (size_t i = 0, i_end = cluster_num_; i < i_end; i ++)
    {
        value_type temp_dist = distance(traj_point, center_trajs_[i]);
        if (min > temp_dist)
        {
            min = temp_dist;
            cluster_id = i;
        }
    }

    return cluster_id;
}

void Trajectories::getPointsFromPath(int id, TrajectoryPoint& traj_point)
{
    TrajectoryPath traj_path = traj_paths_.at(id);
    for (size_t j = 0, j_end = traj_path._trajectory.size(); j < j_end; j ++)
    {
        PointCloud* point_cloud = points_file_system_->getPointCloud(j);
        const Point& point = point_cloud->at(traj_path._trajectory[j]);
        Point center_point;
        center_point.x = point.x;
        center_point.y = point.y;
        center_point.z = point.z;

        traj_point.push_back(center_point);
    }
}

void Trajectories::mean_path(const std::vector<int>& ids, TrajectoryPoint& traj_point)
{
    TrajectoryPoints traj_points;
    for (size_t i = 0, i_end = ids.size(); i < i_end; i ++)
    {
        TrajectoryPoint temp_point;
        getPointsFromPath(ids[i], temp_point);
        traj_points.push_back(temp_point);
    }

    size_t traj_length = traj_points[0].size();
    for (size_t i = 0, i_end = traj_length; i < i_end; i ++)
    {
        Point mean_point;
        value_type x = 0, y = 0, z = 0;

        size_t traj_num = traj_points.size();
        for (size_t j = 0, j_end = traj_num; j < j_end; j ++)
        {
            const Point& point = traj_points[j][i];

            x += point.x;
            y += point.y;
            z += point.z;
        }

        mean_point.x = x / traj_num;
        mean_point.y = y / traj_num;
        mean_point.z = z / traj_num;

        traj_point.push_back(mean_point);
    }
}

bool Trajectories::terminal(const TrajectoryPoints& current_centers, const TrajectoryPoints& next_centers)
{
    const value_type eps = 1e-3;

    for (size_t i = 0; i < cluster_num_; i ++)
    {
        value_type delta = distance(current_centers[i], next_centers[i]);
        if (delta > eps)
            return false;
    }

    return true;
}