
#include <iostream>

#include "main_window.h"
#include "point_cloud.h"
#include "mesh_model.h"
#include "points_file_system.h"
#include "mesh_file_system.h"
#include "scene_widget.h"
#include "tracking_system.h"
#include "track_thread.h"


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

    int start_frame = tracking_system_->points_file_system_->getStartFrame();
    int end_frame = tracking_system_->points_file_system_->getEndFrame();

    PointCloud* tracking_template = tracking_system_->points_file_system_->getPointCloud(start_frame);
    MainWindow::getInstance()->getSceneWidget()->addSceneChild(tracking_template);

    for (int i = start_frame + 1; i <= end_frame; i ++)
    {
        std::cout << "tracking [frame " << i << "]" << std::endl;

        PointCloud* tracked_frame = tracking_system_->points_file_system_->getPointCloud(i);	
        tracking_system_->points_file_system_->showPointCloud(i - 1);
        tracking_template->expire();

        tracking_system_->cpd_registration(*tracked_frame, *tracking_template);

        tracking_system_->points_file_system_->hidePointCloud(i - 1);
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

    int start_frame = tracking_system_->points_file_system_->getStartFrame();
    int end_frame = tracking_system_->points_file_system_->getEndFrame();

    QSet<QPersistentModelIndex> mesh_indexes = tracking_system_->mesh_file_system_->getCheckedIndexes();
    MeshModel* tracking_template = tracking_system_->mesh_file_system_->getMeshModel(mesh_indexes.values().at(0));

    /*std::vector<int> deform_idx;
    for (size_t i = 0, i_end = tracking_template->getVertices()->size(); i < i_end; i ++)
        deform_idx.push_back(i);*/

    for (int i = start_frame + 1; i <= end_frame; i ++)
    {
        std::cout << "tracking [frame " << i << "]" << std::endl;

        PointCloud* tracked_frame = tracking_system_->points_file_system_->getPointCloud(i);	
        tracking_system_->points_file_system_->showPointCloud(i - 1);
        tracking_template->expire();

        tracking_system_->cpd_registration(*tracked_frame, *tracking_template);

       // tracking_template->deform(*tracking_template->getVertices(), deform_idx);
        tracking_template->deform(*tracking_template->getVertices());

        tracking_system_->points_file_system_->hidePointCloud(i - 1);
    }

    std::cout << "Mesh Tracking Finished!" << std::endl;
}
