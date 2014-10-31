
#include "file_system_model.h"
#include "point_cloud.h"

#include "tracking_system.h"

TrackingSystem::TrackingSystem(FileSystemModel* points_file_system, FileSystemModel* mesh_file_system)
{
	points_file_system_ = points_file_system;
	mesh_file_system_ = mesh_file_system;
}

TrackingSystem::~TrackingSystem()
{}


void TrackingSystem::track()
{
	//int start_frame = file_system_model_->getStartFrame();
	//int end_frame = file_system_model_->getEndFrame();

	//for (int i = start_frame; i <= end_frame; i ++)
	//{
	//	PointCloud* tracked_frame = file_system_model_->getPointCloud(i);
	//	
	//	//PointCloud* deform_template = cpd(surface_template_, tracked_frame);

	//}
}

