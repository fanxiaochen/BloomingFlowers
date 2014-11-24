
#include <iostream>

#include "main_window.h"
#include "point_cloud.h"
#include "mesh_model.h"
#include "points_file_system.h"
#include "mesh_file_system.h"
#include "scene_widget.h"
#include "tracking_system.h"
#include "trajectory_model.h"
#include "task_thread.h"


PointsTrackThread::PointsTrackThread(TrackingSystem* tracking_system)
    :QThread()
{
    tracking_system_ = tracking_system;
}

PointsTrackThread::~PointsTrackThread()
{}

void PointsTrackThread::run()
{
    std::cout << "Point Cloud Tracking Starts..." << std::endl;

    PointsFileSystem* points_file_system = tracking_system_->getPointsFileSystem();

    int start_frame = points_file_system->getStartFrame();
    int end_frame = points_file_system->getEndFrame();

    PointCloud* tracking_template = points_file_system->getPointCloud(start_frame);
    MainWindow::getInstance()->getSceneWidget()->addSceneChild(tracking_template);

    for (int i = start_frame + 1; i <= end_frame; i ++)
    {
        std::cout << "tracking [frame " << i << "]" << std::endl;

        PointCloud* tracked_frame = points_file_system->getPointCloud(i);	
        points_file_system->showPointCloud(i - 1);
        tracking_template->expire();

        tracking_system_->cpd_registration(*tracked_frame, *tracking_template);

        points_file_system->hidePointCloud(i - 1);
    }

    std::cout << "Point Cloud Tracking Finished!" << std::endl;
}


MeshTrackThread::MeshTrackThread(TrackingSystem* tracking_system)
    :QThread()
{
    tracking_system_ = tracking_system;
}

MeshTrackThread::~MeshTrackThread()
{}

void MeshTrackThread::run()
{
    std::cout << "Mesh Tracking Starts..." << std::endl;

    PointsFileSystem* points_file_system = tracking_system_->getPointsFileSystem();
    MeshFileSystem* mesh_file_system = tracking_system_->getMeshFileSystem();

    int start_frame = points_file_system->getStartFrame();
    int end_frame = points_file_system->getEndFrame();

    QSet<QPersistentModelIndex> mesh_indexes = mesh_file_system->getCheckedIndexes();
    MeshModel* tracking_template = mesh_file_system->getMeshModel(mesh_indexes.values().at(0));

    /*std::vector<int> deform_idx;
    for (size_t i = 0, i_end = tracking_template->getVertices()->size(); i < i_end; i ++)
        deform_idx.push_back(i);*/

    for (int i = start_frame + 1; i <= end_frame; i ++)
    {
        std::cout << "tracking [frame " << i << "]" << std::endl;

        PointCloud* tracked_frame = points_file_system->getPointCloud(i);	
        points_file_system->showPointCloud(i - 1);
        tracking_template->expire();

        tracking_system_->cpd_registration(*tracked_frame, *tracking_template);

       // tracking_template->deform(*tracking_template->getVertices(), deform_idx);
        tracking_template->deform(*tracking_template->getVertices());

        points_file_system->hidePointCloud(i - 1);
    }

    std::cout << "Mesh Tracking Finished!" << std::endl;
}


TrajectoryTrackThread::TrajectoryTrackThread(TrackingSystem* tracking_system)
    :QThread()
{
    tracking_system_ = tracking_system;
}

TrajectoryTrackThread::~TrajectoryTrackThread()
{}

void TrajectoryTrackThread::run()
{
    std::cout << "Trajectory Tracking Starts..." << std::endl;

    PointsFileSystem* points_file_system = tracking_system_->getPointsFileSystem();
    Trajectories* trajectories = tracking_system_->getTrajectories();

    int start_frame = points_file_system->getStartFrame();
    int end_frame = points_file_system->getEndFrame();

    PointCloud* source = points_file_system->getPointCloud(start_frame);
    points_file_system->showPointCloud(start_frame);
    trajectories->showTrajectories();

    std::vector<int> src_idx;
    for (size_t i = 0, i_end = source->size(); i < i_end; i ++)
    {
        src_idx.push_back(i);

        Trajectories::TrajectoryPath traj_path;
        traj_path._trajectory.push_back(i);
        traj_path._id = i;
        traj_path._label = -1;
        trajectories->getPaths().push_back(traj_path);
    }

    for (int i = start_frame + 1; i <= end_frame; i ++)
    {
        std::cout << "tracking [frame " << i << "]" << std::endl;

        PointCloud* target = points_file_system->getPointCloud(i);
        points_file_system->showPointCloud(i);

        std::vector<int> tar_idx;

        tracking_system_->cpd_registration(*source, *target, src_idx, tar_idx);

        trajectories->expire();

        //points_file_system->hidePointCloud(i - 1);

        source = target;
        src_idx = tar_idx;
    }

    //for (auto& it = trajectories->getPaths().begin(); it != trajectories->getPaths().end(); ++ it)
    //{
    //    std::cout << "id " << it->_id << std::endl;
    //    for (auto& tt = it->_trajectory.begin(); tt != it->_trajectory.end(); ++ tt)
    //        std::cout << *tt << " ";
    //    std::cout << std::endl;
    //}

    std::string workspace = MainWindow::getInstance()->getWorkspace();
    std::string traj_file = workspace + "/trajectories.json";
    if (!trajectories->save(traj_file))
        std::cerr << "trajectories save error!" << std::endl;

    std::cout << "Trajectory Tracking Finished!" << std::endl;
}


TrajClusteringThread::TrajClusteringThread(Trajectories* trajectories)
    :QThread()
{
    trajectories_ = trajectories;
}

TrajClusteringThread::~TrajClusteringThread()
{}

void TrajClusteringThread::run()
{
    std::cout << "Start Trajectory Clustering..." << std::endl;

    trajectories_->clustering();

    trajectories_->expire();

    std::cout << "Trajectory Clustering Finished..." << std::endl;
    return;
}