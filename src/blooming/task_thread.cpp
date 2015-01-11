
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
#include "task_thread.h"
#include "deform_model.h"


EMTrackThread::EMTrackThread(TrackingSystem* tracking_system)
    :QThread()
{
    tracking_system_ = tracking_system;
}

EMTrackThread::~EMTrackThread()
{}

void EMTrackThread::run()
{
    std::cout << "EM Tracking Starts..." << std::endl;

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

    Flower* forward_flower = new Flower(*flower);
    Flower* backward_flower = new Flower(*flower);

    std::string workspace = MainWindow::getInstance()->getWorkspace();
    QDir mesh_dir(QString(workspace.c_str()));
    mesh_dir.mkdir("meshes");

    std::cout << "Forward Tracking..." << std::endl;
    forward_flower->show();
    for (size_t i = key_frame , i_end = end_frame;
        i <= i_end; ++ i)
    {
        std::cout << "tracking [frame " << i << "]" << std::endl;

        // EM tracking 
        PointCloud* forward_cloud = points_file_system->getPointCloud(i);
        tracking_system_->em_registration(*forward_cloud, *forward_flower);

        QString frame_path = mesh_dir.absolutePath() + "/meshes";
        QDir mesh_frame(frame_path);
        QString frame_file = QString("frame_%1").arg(i, 5, 10, QChar('0'));
        mesh_frame.mkdir(frame_file);
        QString mesh_path = mesh_frame.absolutePath() + "/" + frame_file;
        forward_flower->save(mesh_path.toStdString());

        forward_flower->update();

        points_file_system->hidePointCloud(i - 1);
        points_file_system->showPointCloud(i);
    }
    //   forward_flower->hide();

    std::cout << "EM Tracking Finished!" << std::endl;
}
