#ifndef TRAJECTORY_MODEL_H
#define TRAJECTORY_MODEL_H

#include <string>
#include <vector>

class PointsFileSystem;

class Trajectories
{
public:
    typedef std::vector<int> TrajectoryPath;
    typedef std::vector<TrajectoryPath> TrajectoryPaths;

public:
    Trajectories(PointsFileSystem* points_file_system);
    ~Trajectories();

    void load(const std::string& file);
    void save(const std::string& file);

    void build();

    inline TrajectoryPath& getPath(int index) { return traj_path_[index]; }
    inline TrajectoryPaths& getPaths(){ return traj_path_; }


private:
   PointsFileSystem*    points_file_system_;

   TrajectoryPaths      traj_path_;
};

#endif