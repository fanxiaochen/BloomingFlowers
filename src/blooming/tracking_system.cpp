

#include <core/cpd_nonrigid.hpp>

#include "types_wrapper.h"

#include "point_cloud.h"
#include "mesh_model.h"
#include "track_thread.h"
#include "tracking_system.h"

TrackingSystem::TrackingSystem(PointsFileSystem* points_file_system, MeshFileSystem* mesh_file_system)
{
	points_file_system_ = points_file_system;
	mesh_file_system_ = mesh_file_system;	
}

TrackingSystem::~TrackingSystem()
{}


void TrackingSystem::pointcloud_tracking()
{
    PointsTrackThread* track_thread = new PointsTrackThread(this);
    connect(track_thread, SIGNAL(finished()), track_thread, SLOT(quit()));

    track_thread->start();
    return;
}

void TrackingSystem::mesh_tracking()
{
    MeshTrackThread* track_thread = new MeshTrackThread(this);
    connect(track_thread, SIGNAL(finished()), track_thread, SLOT(quit()));

    track_thread->start();
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
	reg->setEnergyTolerance(1e-5);
	reg->setOutlierWeight(0.1);
	reg->setFgtFlag(true);
    reg->setFgtEpsilon(1e-4);
    reg->setLowRankFlag(true);
    reg->setKLowRank(100);
	reg->run();

	MATRIX_TO_POINTCLOUD(reg->getModel(), tracking_template);
}

void TrackingSystem::cpd_registration(const PointCloud& tracked_frame, MeshModel& tracking_template)
{	
	PointMatrix tracked_pm = POINTCLOUD_TO_MATRIX(tracked_frame);
	PointMatrix tracking_pm = MESHMODEL_TO_MATRIX(tracking_template);

	cpd::CPDNRigid<value_type, 3>* reg = new cpd::CPDNRigid<value_type, 3>();

	reg->setInputData(tracking_pm, tracked_pm);
	reg->setVision(false);
	reg->setIterativeNumber(50);
	reg->setVarianceTolerance(1e-5);
	reg->setEnergyTolerance(1e-3);
	reg->setOutlierWeight(0.1);
    reg->setFgtFlag(true);
    reg->setFgtEpsilon(1e-3);
    reg->setLowRankFlag(true);
    reg->setKLowRank(40);
    reg->run();

	MATRIX_TO_MESHMODEL(reg->getModel(), tracking_template);
}


