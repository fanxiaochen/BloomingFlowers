#ifndef TRAJECTORY_MODEL_H
#define TRAJECTORY_MODEL_H

#include <string>
#include <vector>

#include "types_wrapper.h"

class PointsFileSystem;

class Trajectories
{
public:
    typedef std::vector<Point> TrajectoryPoint;
    typedef std::vector<TrajectoryPoint> TrajectoryPoints;

    typedef struct
    {
        int _id;
        std::vector<int> _trajectory;
        int _label;
    }TrajectoryPath;

    typedef std::vector<TrajectoryPath> TrajectoryPaths;
   
public:
    Trajectories(PointsFileSystem* points_file_system);
    ~Trajectories();

    void load(const std::string& file);
    void save(const std::string& file);

    void clustering();

    void build();

    inline TrajectoryPath& getPath(int index) { return traj_paths_[index]; }
    inline TrajectoryPaths& getPaths(){ return traj_paths_; }

private:
    value_type distance(const TrajectoryPath& path_1, const TrajectoryPath& path_2);
    value_type distance(const TrajectoryPoint& point_1, const TrajectoryPoint& point_2);
    void getPointsFromPath(int id, TrajectoryPoint& traj_point);

    int determineCluster(const TrajectoryPath& path);
    void mean_path(const std::vector<int>& ids, TrajectoryPoint& traj_point);
    bool terminal(const TrajectoryPoints& current_centers, const TrajectoryPoints& next_centers);
    void k_means();

private:
   PointsFileSystem*    points_file_system_;
   TrajectoryPaths      traj_paths_;

   int                  cluster_num_;
   TrajectoryPoints     center_trajs_;
};

#endif