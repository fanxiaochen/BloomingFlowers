
#include "point_cloud.h"
#include "mesh_model.h"
#include "flower.h"
#include "task_thread.h"
#include "tracking_system.h"
#include "deform_model.h"
#include "elastic_model.h"
#include "weighted_model.h"
#include "solver.h"

TrackingSystem::TrackingSystem(PointsFileSystem* points_file_system)
    :key_frame_(-1)
{
    points_file_system_ = points_file_system;

    start_frame_ = points_file_system_->getStartFrame();
    end_frame_ = points_file_system_->getEndFrame();
}

TrackingSystem::TrackingSystem(PointsFileSystem* points_file_system, MeshFileSystem* mesh_file_system)
    :key_frame_(-1)
{
	points_file_system_ = points_file_system;
	mesh_file_system_ = mesh_file_system;	
    
    start_frame_ = points_file_system_->getStartFrame();
    end_frame_ = points_file_system_->getEndFrame();
}

TrackingSystem::~TrackingSystem()
{}

void TrackingSystem::em_arap()
{
    EATrackThread* track_thread = new EATrackThread(this);
    connect(track_thread, SIGNAL(finished()), track_thread, SLOT(quit()));

    track_thread->start();
    return;
}


void TrackingSystem::em_elastic()
{
	EETrackThread* track_thread = new EETrackThread(this);
	connect(track_thread, SIGNAL(finished()), track_thread, SLOT(quit()));

	track_thread->start();
	return;
}

void TrackingSystem::wem_arap()
{
    WEATrackThread* track_thread = new WEATrackThread(this);
    connect(track_thread, SIGNAL(finished()), track_thread, SLOT(quit()));

    track_thread->start();
    return;
}

void TrackingSystem::lbs_arap()
{
    LATrackThread* track_thread = new LATrackThread(this);
    connect(track_thread, SIGNAL(finished()), track_thread, SLOT(quit()));

    track_thread->start();
    return;
}

void TrackingSystem::detectTip()
{
    TipThread* track_thread = new TipThread(this);
    connect(track_thread, SIGNAL(finished()), track_thread, SLOT(quit()));

    track_thread->start();
    return;
}

void TrackingSystem::detectBoundary()
{
    BoundaryThread* track_thread = new BoundaryThread(this);
    connect(track_thread, SIGNAL(finished()), track_thread, SLOT(quit()));

    track_thread->start();
    return;
}


// em + arap registration
void TrackingSystem::ea_registration(PointCloud& tracked_frame, Flower& tracking_template)
{
    DeformModel deform_model(&tracked_frame, &tracking_template);
    deform_model.deform();
}

// wem + arap registration
void TrackingSystem::wea_registration(PointCloud& tracked_frame, Flower& tracking_template)
{
    WeightedModel weighted_model(&tracked_frame, &tracking_template);
    weighted_model.deform();
}

void TrackingSystem::ee_registration( PointCloud& tracked_frame, Flower& tracking_template )
{
	ElasticModel elastic_model(&tracked_frame, &tracking_template);
	elastic_model.deform();
}

// em + lbs + arap registration
void TrackingSystem::la_registration( PointCloud& tracked_frame, Flower& tracking_template )
{
    Solver solver(&tracked_frame, &tracking_template);

    //// first stage -- global boundary tracking
    //solver.init_global_deform();
    //solver.deform();
    //tracking_template.update();
    // second stage -- local inner tracking
    /*solver.init_local_deform();
    solver.deform();
    tracking_template.update();*/

    solver.boundary_inner_setting();
    solver.full_deform();
}
