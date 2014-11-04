
#include "lily.h"

#include <core/cpd_nonrigid.hpp>

#include "point_cloud.h"
#include "mesh_model.h"
#include "tracking_system.h"

TrackingSystem::TrackingSystem(PointsFileSystem* points_file_system, MeshFileSystem* mesh_file_system)
{
	points_file_system_ = points_file_system;
	mesh_file_system_ = mesh_file_system;
	track_thread_ = new TrackThread(this);

	connect(track_thread_, SIGNAL(finished()), track_thread_, SLOT(quit()));
}

TrackingSystem::~TrackingSystem()
{}


void TrackingSystem::track()
{
	track_thread_->start();
	return;
}

void TrackingSystem::cpd_registration(const PointCloud& tracked_frame, PointCloud& tracking_template)
{	
	PointMatrix tracked_pm = POINTCLOUD_TO_MATRIX(tracked_frame);
	PointMatrix tracking_pm = POINTCLOUD_TO_MATRIX(tracking_template);

	cpd::CPDNRigid<value_type, 3>* reg = new cpd::CPDNRigid<value_type, 3>();

	reg->setInputData(tracking_pm, tracked_pm);
	reg->setVision(false);
	reg->setIterativeNumber(50);
	reg->setVarianceTolerance(1e-5);
	reg->setEnergyTolerance(1e-3);
	reg->setOutlierWeight(0.1);
	reg->setFgtFlag(true);
	/*reg->setLowRankFlag(true);
	reg->setKLowRank(10);*/
	reg->run();

	MATRIX_TO_POINTCLOUD(reg->getModel(), tracking_template);
}

void TrackingSystem::cpd_registration(const PointCloud& tracked_frame, MeshModel& tracking_template)
{	
	PointMatrix tracked_pm = POINTCLOUD_TO_MATRIX(tracked_frame);
	PointMatrix tracking_pm = MESHMODEL_TO_MATRIX(tracking_template);

	cpd::CPDNRigid<value_type, 3>* reg = new cpd::CPDNRigid<value_type, 3>();

	std::cout << tracking_pm << std::endl;

	reg->setInputData(tracking_pm, tracked_pm);
	reg->setVision(false);
	reg->setIterativeNumber(50);
	reg->setVarianceTolerance(1e-5);
	reg->setEnergyTolerance(1e-3);
	reg->setOutlierWeight(0.1);
	reg->setFgtFlag(true);
	/*reg->setLowRankFlag(true);
	reg->setKLowRank(10);*/
	reg->run();

	MATRIX_TO_MESHMODEL(reg->getModel(), tracking_template);
}


TrackThread::TrackThread(TrackingSystem* tracking_system)
	:QThread()
{
	tracking_system_ = tracking_system;
}

TrackThread::~TrackThread()
{}

void TrackThread::run()
{
	std::cout << "Tracking Thread Starts..." << std::endl;

	int start_frame = tracking_system_->points_file_system_->getStartFrame();
	int end_frame = tracking_system_->points_file_system_->getEndFrame();

	QSet<QPersistentModelIndex> mesh_indexes = tracking_system_->mesh_file_system_->getCheckedIndexes();
	MeshModel* tracking_template = tracking_system_->mesh_file_system_->getMeshModel(mesh_indexes.values().at(0));
	//PointCloud* tracking_template = tracking_system_->points_file_system_->getPointCloud(start_frame);
	//MainWindow::getInstance()->getSceneWidget()->addSceneChild(tracking_template);

	for (int i = start_frame + 5; i <= end_frame; i ++)
	{
		std::cout << "tracking [frame " << i << "]" << std::endl;

		PointCloud* tracked_frame = tracking_system_->points_file_system_->getPointCloud(i);	
		tracking_system_->points_file_system_->showPointCloud(i - 1);
		tracking_template->expire();

		tracking_system_->cpd_registration(*tracked_frame, *tracking_template);

		//std::cout << tracking_template->getVertices()->size() << std::endl;
		
		tracking_system_->points_file_system_->hidePointCloud(i - 1);
	}

	std::cout << "Tracking Finished!" << std::endl;
}
