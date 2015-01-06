#include <core/cpd_nonrigid.hpp>

#include "types_wrapper.h"

#include "point_cloud.h"
#include "mesh_model.h"
#include "flower.h"
#include "task_thread.h"
#include "trajectory_model.h"
#include "tracking_system.h"
#include "deform_model.h"

TrackingSystem::TrackingSystem(PointsFileSystem* points_file_system)
    :trajectories_(new Trajectories(points_file_system)), key_frame_(-1), flowers_(new Flowers)
{
    points_file_system_ = points_file_system;

    start_frame_ = points_file_system_->getStartFrame();
    end_frame_ = points_file_system_->getEndFrame();
}

TrackingSystem::TrackingSystem(PointsFileSystem* points_file_system, MeshFileSystem* mesh_file_system)
    :trajectories_(new Trajectories(points_file_system)), key_frame_(-1), flowers_(new Flowers)
{
	points_file_system_ = points_file_system;
	mesh_file_system_ = mesh_file_system;	
    
    start_frame_ = points_file_system_->getStartFrame();
    end_frame_ = points_file_system_->getEndFrame();
}

TrackingSystem::~TrackingSystem()
{}


void TrackingSystem::cpd_registration(const PointCloud& tracked_frame, PointCloud& tracking_template)
{	
    PointMatrix tracked_pm = POINTCLOUD_TO_MATRIX(tracked_frame);
    PointMatrix tracking_pm = POINTCLOUD_TO_MATRIX(tracking_template);

    cpd::CPDNRigid<float, 3>* reg = new cpd::CPDNRigid<float, 3>();

    reg->setInputData(tracking_pm, tracked_pm);
    reg->setVision(false);
    reg->setIterativeNumber(50);
    reg->setVarianceTolerance(1e-5);
    reg->setEnergyTolerance(1e-5);
    reg->setOutlierWeight(0.1);
    reg->setFgtFlag(true);
    reg->setFgtEpsilon(1e-4);
    //   reg->setLowRankFlag(true);
    //   reg->setKLowRank(100);
    reg->run();

    MATRIX_TO_POINTCLOUD(reg->getModel(), tracking_template);
}

void TrackingSystem::cpd_registration(const PointCloud& tracked_frame, MeshModel& tracking_template)
{	
    PointMatrix tracked_pm = POINTCLOUD_TO_MATRIX(tracked_frame);
    PointMatrix tracking_pm = MESHMODEL_TO_MATRIX(tracking_template);

    cpd::CPDNRigid<float, 3>* reg = new cpd::CPDNRigid<float, 3>();

    reg->setInputData(tracking_pm, tracked_pm);
    reg->setVision(false);
    reg->setIterativeNumber(50);
    reg->setVarianceTolerance(1e-5);
    reg->setEnergyTolerance(1e-3);
    reg->setOutlierWeight(0.1);
    reg->setFgtFlag(true);
    reg->setFgtEpsilon(1e-3);
    //    reg->setLowRankFlag(true);
    //    reg->setKLowRank(40);
    reg->run();

    MATRIX_TO_MESHMODEL(reg->getModel(), tracking_template);
}

void TrackingSystem::cpd_registration(const PointCloud& source_frame, const PointCloud& target_frame, 
                                      const std::vector<int>& src_idx, std::vector<int>& tar_idx)
{
    PointMatrix source_pm;
    source_pm.resize(src_idx.size(), 3);

    for (size_t i = 0, i_end = src_idx.size(); i < i_end; i ++)
    {
        const Point& point = source_frame.at(src_idx[i]);
        source_pm(i, 0) = point.x;
        source_pm(i, 1) = point.y;
        source_pm(i, 2) = point.z;
    }

    PointMatrix target_pm = POINTCLOUD_TO_MATRIX(target_frame);

    cpd::CPDNRigid<float, 3>* reg = new cpd::CPDNRigid<float, 3>();

    reg->setInputData(source_pm, target_pm);
    reg->setVision(false);
    reg->setIterativeNumber(50);
    reg->setVarianceTolerance(1e-5);
    reg->setEnergyTolerance(1e-3);
    reg->setOutlierWeight(0.1);
    reg->setFgtFlag(true);
    reg->setFgtEpsilon(1e-3);
    //    reg->setLowRankFlag(true);
    //    reg->setKLowRank(40);
    reg->run();

    CorresMatrix corres = reg->getCorrespondences();
    CorresMatrix::Index max_index;

    for (size_t i = 0, i_end = corres.rows(); i < i_end; i ++)
    {
        corres.row(i).maxCoeff(&max_index);
        tar_idx.push_back(int(max_index));

        trajectories_->getPath(i)._trajectory.push_back(int(max_index));
    }    
}

void TrackingSystem::cpd_flower(const PointCloud& tracked_frame, Flower& tracking_template)
{	
    PointMatrix tracked_pm = POINTCLOUD_TO_MATRIX(tracked_frame);
    PointMatrix tracking_pm = FLOWER_TO_MATRIX(tracking_template);

    cpd::CPDNRigid<float, 3>* reg = new cpd::CPDNRigid<float, 3>();

    reg->setInputData(tracking_pm, tracked_pm);
    reg->setVision(false);
    reg->setIterativeNumber(100);
    reg->setVarianceTolerance(1e-5);
    reg->setEnergyTolerance(1e-3);
    reg->setOutlierWeight(0.1);
    reg->setFgtFlag(true);
    reg->setFgtEpsilon(1e-3);
    //    reg->setLowRankFlag(true);
    //    reg->setKLowRank(40);
    reg->run();

    MATRIX_TO_FLOWER(reg->getModel(), tracking_template);
}

void TrackingSystem::cpd_petal(const PointCloud& tracked_frame, Petal& tracking_template)
{	
    PointMatrix tracked_pm = POINTCLOUD_TO_MATRIX(tracked_frame);
    PointMatrix tracking_pm = PETAL_TO_MATRIX(tracking_template);

    cpd::CPDNRigid<float, 3>* reg = new cpd::CPDNRigid<float, 3>();

    reg->setInputData(tracking_pm, tracked_pm);
    reg->setVision(false);
    reg->setIterativeNumber(50);
    reg->setVarianceTolerance(1e-5);
    reg->setEnergyTolerance(1e-3);
    reg->setOutlierWeight(0.1);
    reg->setFgtFlag(true);
    reg->setFgtEpsilon(1e-3);
    //    reg->setLowRankFlag(true);
    //    reg->setKLowRank(40);
    reg->run();

    MATRIX_TO_PETAL(reg->getModel(), tracking_template);
}



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

void TrackingSystem::petal_tracking()
{
    PetalTrackThread* track_thread = new PetalTrackThread(this);
    connect(track_thread, SIGNAL(finished()), track_thread, SLOT(quit()));

    track_thread->start();
    return;
}

void TrackingSystem::flower_tracking()
{
    FlowerTrackThread* track_thread = new FlowerTrackThread(this);
    connect(track_thread, SIGNAL(finished()), track_thread, SLOT(quit()));

    track_thread->start();
    return;
}

void TrackingSystem::buildTrajectories()
{
    TrajectoryTrackThread* track_thread = new TrajectoryTrackThread(this);
    connect(track_thread, SIGNAL(finished()), track_thread, SLOT(quit()));

    track_thread->start();
    return;
}

void TrackingSystem::clusterTrajectories()
{
    TrajClusteringThread* clustering_thread = new TrajClusteringThread(trajectories_);
    connect(clustering_thread, SIGNAL(finished()), clustering_thread, SLOT(quit()));

    clustering_thread->start();
    return;
}

void TrackingSystem::template_segmentation()
{
    TemplateSegmentThread* temseg_thread = new TemplateSegmentThread(this);
    connect(temseg_thread, SIGNAL(finished()), temseg_thread, SLOT(quit()));

    temseg_thread->start();
    return;
}




void TrackingSystem::em_tracking()
{
    EMTrackThread* track_thread = new EMTrackThread(this);
    connect(track_thread, SIGNAL(finished()), track_thread, SLOT(quit()));

    track_thread->start();
    return;
}

void TrackingSystem::em_registration(PointCloud& tracked_frame, Flower& tracking_template)
{
    DeformModel deform_model(&tracked_frame, &tracking_template);
    deform_model.deform();
}
