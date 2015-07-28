#ifndef TRAJECTORY_MODEL_H
#define TRAJECTORY_MODEL_H
#include <vector>

#include "point_cloud.h"
#include "renderable.h"

class TrajectoryModel: public Renderable
{
public:
    typedef std::vector<Point> Trajectory;
    typedef std::vector<Trajectory> Trajectories;

private:
    Trajectories    trajectories_;
};
#endif