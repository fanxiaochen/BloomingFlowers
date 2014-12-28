
#include <iostream>

#include "main_window.h"
#include "parameters.h"
#include "point_cloud.h"
#include "flower.h"
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

    for (int i = start_frame + 1; i <= end_frame; i ++)
    {
        std::cout << "tracking [frame " << i << "]" << std::endl;

        PointCloud* tracked_frame = points_file_system->getPointCloud(i);	
        points_file_system->showPointCloud(i - 1);

        tracking_system_->cpd_registration(*tracked_frame, *tracking_template);

        tracking_template->expire();
        points_file_system->hidePointCloud(i - 1);
    }

    std::cout << "Mesh Tracking Finished!" << std::endl;
}

PetalTrackThread::PetalTrackThread(TrackingSystem* tracking_system)
    :QThread()
{
    tracking_system_ = tracking_system;
}

PetalTrackThread::~PetalTrackThread()
{}

void PetalTrackThread::run()
{
    std::cout << "Petal Tracking Starts..." << std::endl;

    PointsFileSystem* points_file_system = tracking_system_->getPointsFileSystem();
    MeshFileSystem* mesh_file_system = tracking_system_->getMeshFileSystem();

    int start_frame = points_file_system->getStartFrame();
    int end_frame = points_file_system->getEndFrame();

    QSet<QPersistentModelIndex> mesh_indexes = mesh_file_system->getCheckedIndexes();
    MeshModel* tracking_template = mesh_file_system->getMeshModel(mesh_indexes.values().at(0));

    Petal* petal = new Petal(*tracking_template);


    std::string workspace = MainWindow::getInstance()->getWorkspace();
    QDir mesh_dir(QString(workspace.c_str()));
    mesh_dir.mkdir("meshes");
    
    for (size_t i = start_frame, i_end = end_frame;
        i <= i_end; ++ i)
    {
        std::cout << "tracking [frame " << i << "]" << std::endl;
    
        PointCloud* forward_cloud = points_file_system->getPointCloud(i);
    
        tracking_system_->cpd_petal(*forward_cloud, *petal);
    
        osg::ref_ptr<osg::Vec3Array>& hard_ctrs = petal->getHardCtrs();
        std::vector<int>& hard_idx = petal->getHardCtrsIndex();
       
        petal->deform(*hard_ctrs, hard_idx);
    
        QString frame_path = mesh_dir.absolutePath() + "/meshes";
        QDir mesh_frame(frame_path);
        QString frame_file = QString("frame_%1").arg(i, 5, 10, QChar('0'));
        mesh_frame.mkdir(frame_file);
        QString mesh_path = mesh_frame.absolutePath() + "/" + frame_file;
        petal->save(mesh_path.toStdString());
    
        petal->expire();
    
        points_file_system->hidePointCloud(i - 1);
        points_file_system->showPointCloud(i);
    }

    std::cout << "Petal Tracking Finished!" << std::endl;
}

FlowerTrackThread::FlowerTrackThread(TrackingSystem* tracking_system)
    :QThread()
{
    tracking_system_ = tracking_system;
}

FlowerTrackThread::~FlowerTrackThread()
{}

void FlowerTrackThread::run()
{
    std::cout << "Flower Tracking Starts..." << std::endl;

    int key_frame = MainWindow::getInstance()->getParameters()->getKeyFrame();
    Flowers* flowers = tracking_system_->getFlowers();

    PointsFileSystem* points_file_system = tracking_system_->getPointsFileSystem();
    MeshFileSystem* mesh_file_system = tracking_system_->getMeshFileSystem();

    int start_frame = points_file_system->getStartFrame();
    int end_frame = points_file_system->getEndFrame();

    QSet<QPersistentModelIndex> mesh_indexes = mesh_file_system->getCheckedIndexes();
    
    // build flower structure, memory leak...
    Flower* flower = new Flower;
    for (size_t i = 0, i_end = mesh_indexes.size(); i < i_end; ++ i)
    {
        osg::ref_ptr<Petal> petal_template = mesh_file_system->getMeshModel(mesh_indexes.values().at(i));
        flower->getPetals().push_back(*petal_template); // deep copy
        mesh_file_system->hideMeshModel(mesh_indexes.values().at(i));
    }

    flower->show();

//    flower->determineVisibility();
//    flower->update();
//    flower->buildHardCtrs(2);

    Flower* forward_flower = flower;
 //   Flower* backward_flower = flower;

    std::string workspace = MainWindow::getInstance()->getWorkspace();
    QDir mesh_dir(QString(workspace.c_str()));
    mesh_dir.mkdir("meshes");
//
    std::cout << "Forward Tracking..." << std::endl;
    for (size_t i = key_frame + 1, i_end = end_frame;
        i <= i_end; ++ i)
    {
        std::cout << "tracking [frame " << i << "]" << std::endl;

        forward_flower->determineVisibility();
//
        PointCloud* forward_cloud = points_file_system->getPointCloud(i);
        forward_cloud->template_segmentation(forward_flower);

        forward_flower->buildHardCtrs(2);

//        tracking_system_->cpd_registration(*forward_cloud, *forward_flower);

        std::vector<osg::ref_ptr<osg::Vec3Array> > hard_ctrs;
        std::vector<std::vector<int> > hard_idx;
        for (size_t j = 0, j_end = forward_flower->getPetals().size(); j < j_end; ++ j)
        {
            osg::ref_ptr<PointCloud> petal_cloud = forward_cloud->getPetalCloud(j);
            Petal& petal = forward_flower->getPetals().at(j);
            tracking_system_->cpd_petal(*petal_cloud, petal);

            hard_ctrs.push_back(petal.getHardCtrs());
            hard_idx.push_back(petal.getHardCtrsIndex());
        }

        forward_flower->deform(hard_ctrs, hard_idx);

        QString frame_path = mesh_dir.absolutePath() + "/meshes";
        QDir mesh_frame(frame_path);
        QString frame_file = QString("frame_%1").arg(i, 5, 10, QChar('0'));
        mesh_frame.mkdir(frame_file);
        QString mesh_path = mesh_frame.absolutePath() + "/" + frame_file;
        forward_flower->save(mesh_path.toStdString());

        forward_flower->update();
////        flowers->push_back(*flower);

        points_file_system->hidePointCloud(i - 1);
        points_file_system->showPointCloud(i);
    }

//
//    std::cout << "Backward Tracking..." << std::endl;
//    for (size_t i = key_frame - 1, i_end = start_frame;
//        i >= i_end; -- i)
//    {
//        std::cout << "tracking [frame " << i << "]" << std::endl;
//
//        PointCloud* backward_cloud = points_file_system->getPointCloud(i);
//
//        tracking_system_->cpd_registration(*backward_cloud, *backward_flower);
//
//        std::vector<osg::ref_ptr<osg::Vec3Array> > hard_ctrs;
//        std::vector<std::vector<int> > hard_idx;
//        for (size_t j = 0, j_end = backward_flower->getPetals().size(); j < j_end; ++ j)
//        {
//            Petals& petals = backward_flower->getPetals();
//            hard_ctrs.push_back(petals[j].getHardCtrs());
//            hard_idx.push_back(petals[j].getHardCtrsIndex());
//        }
//
//        backward_flower->deform(hard_ctrs, hard_idx);
//
//        QString frame_path = mesh_dir.absolutePath() + "/meshes";
//        QDir mesh_frame(frame_path);
//        QString frame_file = QString("frame_%1").arg(i, 5, 10, QChar('0'));
//        mesh_frame.mkdir(frame_file);
//        QString mesh_path = mesh_frame.absolutePath() + "/" + frame_file;
//        backward_flower->save(mesh_path.toStdString());
//
//        backward_flower->update();
////        flowers->push_back(*flower);
//
//        points_file_system->hidePointCloud(i + 1);
//        points_file_system->showPointCloud(i);
//    }
//    
//
    std::cout << "Flower Tracking Finished!" << std::endl;
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

KmeansSegmentThread::KmeansSegmentThread(PointsFileSystem* points_file_system, int frame)
    :QThread()
{
    points_file_system_ = points_file_system;
    frame_ = frame;
}

KmeansSegmentThread::~KmeansSegmentThread()
{}

void KmeansSegmentThread::run()
{
    std::cout << "Start Segmentation..." << std::endl;

    points_file_system_->segmentPointCloudByKmeans(frame_);

    std::cout << "Segmentation Finished..." << std::endl;
    return;
}

TemplateSegmentThread::TemplateSegmentThread(TrackingSystem* tracking_system)
    :QThread()
{
    tracking_system_ = tracking_system;
}

TemplateSegmentThread::~TemplateSegmentThread()
{}

void TemplateSegmentThread::run()
{
    std::cout << "Start Template Segmentation..." << std::endl;

    int key_frame = MainWindow::getInstance()->getParameters()->getKeyFrame();

    PointsFileSystem* points_file_system = tracking_system_->getPointsFileSystem();
    MeshFileSystem* mesh_file_system = tracking_system_->getMeshFileSystem();

    int start_frame = points_file_system->getStartFrame();
    int end_frame = points_file_system->getEndFrame();

    QSet<QPersistentModelIndex> mesh_indexes = mesh_file_system->getCheckedIndexes();

    // build flower structure, memory leak...
    Flower* flower = new Flower;
    for (size_t i = 0, i_end = mesh_indexes.size(); i < i_end; ++ i)
    {
        osg::ref_ptr<Petal> petal_template = mesh_file_system->getMeshModel(mesh_indexes.values().at(i));
        flower->getPetals().push_back(*petal_template); // deep copy
        mesh_file_system->hideMeshModel(mesh_indexes.values().at(i));
    }

    flower->show();

    PointCloud* key_cloud = points_file_system->getPointCloud(key_frame);
    key_cloud->template_segmentation(flower);
    key_cloud->expire();

    std::cout << "Template Segmentation Finished..." << std::endl;
    return;
}