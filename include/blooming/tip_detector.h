#ifndef TIP_DETETOR_H
#define TIP_DETETOR_H
#include <vector>

#include "point_cloud.h"
#include "trajectory_model.h"

class TipDetector
{
public:
	TipDetector();

	void setPointCloud(PointCloud* point_cloud);
	void setTrajectoryModel(TrajectoryModel* trajectory_model);

	void detect();


private:
	PointCloud*				point_cloud_;
	TrajectoryModel*		trajectory_model_;

};
#endif