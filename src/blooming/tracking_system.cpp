
#include "core/cpd_nonrigid.hpp"

#include "main_window.h"
#include "scene_widget.h"
#include "points_file_system.h"
#include "mesh_file_system.h"
#include "point_cloud.h"
#include "types_wrapper.h"

#include "tracking_system.h"

TrackingSystem::TrackingSystem(PointsFileSystem* points_file_system, MeshFileSystem* mesh_file_system)
{
	points_file_system_ = points_file_system;
	mesh_file_system_ = mesh_file_system;
}

TrackingSystem::~TrackingSystem()
{}


void TrackingSystem::track()
{
	//Mesh* tracking_template = mesh_file_system_->getMesh();
	PointCloud* tracking_template = points_file_system_->getPointCloud(1);
	MainWindow::getInstance()->getSceneWidget()->addSceneChild(tracking_template);

	int start_frame = points_file_system_->getStartFrame();
	int end_frame = points_file_system_->getEndFrame();

	for (int i = start_frame + 2; i <= end_frame - 23; i ++)
	{
		PointCloud* tracked_frame = points_file_system_->getPointCloud(i);	
		//points_file_system_->showPointCloud(i);

		cpd_registration(*tracked_frame, *tracking_template);
		
		tracking_template->expire();
		//points_file_system_->hidePointCloud(i);
	}
	
	return;
}

void TrackingSystem::cpd_registration(const PointCloud& tracked_frame, PointCloud& tracking_template)
{	
	PointMatrix tracked_pm = POINTCLOUD_TO_MATRIX(tracked_frame);
	PointMatrix tracking_pm = POINTCLOUD_TO_MATRIX(tracking_template);

	cpd::CPDNRigid<value_type, 3>* reg = new cpd::CPDNRigid<value_type, 3>();

	reg->setInputData(tracking_pm, tracked_pm);
	reg->setVision(false);
	reg->setIterativeNumber(100);
	reg->setVarianceTolerance(1e-5);
	reg->setEnergyTolerance(1e-3);
	reg->setOutlierWeight(0.1);
	reg->setFgtFlag(true);
	/*reg->setLowRankFlag(true);
	reg->setKLowRank(10);*/
	reg->run();

	MATRIX_TO_POINTCLOUD(reg->getModel(), tracking_template);
}

