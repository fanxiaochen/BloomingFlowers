#ifndef TRAJECTORY_MODEL_H
#define TRAJECTORY_MODEL_H

#include <string>
#include <vector>

#include "types_wrapper.h"

class PointsFileSystem;

class Trajectories
{
public:
    typedef struct
    {
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

    inline TrajectoryPath& getPath(int index) { return traj_path_[index]; }
    inline TrajectoryPaths& getPaths(){ return traj_path_; }

private:
    value_type distance(const TrajectoryPath& path_1, const TrajectoryPath& path_2);
    void k_means();


private:
   PointsFileSystem*    points_file_system_;

   TrajectoryPaths      traj_path_;
   int                  cluster_num_;
};

#endif