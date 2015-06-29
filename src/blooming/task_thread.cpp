
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


EATrackThread::EATrackThread(TrackingSystem* tracking_system)
    :QThread()
{
    tracking_system_ = tracking_system;
}

EATrackThread::~EATrackThread()
{}

void EATrackThread::run()
{
    std::cout << "EM + ARAP Tracking Starts..." << std::endl;

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

    Flower* forward_flower = new Flower(*flower);
    //Flower* backward_flower = new Flower(*flower);

    std::string workspace = MainWindow::getInstance()->getWorkspace();
    QDir flowers_dir(QString(workspace.c_str()));
    flowers_dir.mkdir("flowers");
    std::string flowers_folder = flowers_dir.absolutePath().toStdString() + "/flowers";


    std::cout << "Forward Tracking..." << std::endl;
    forward_flower->show();
    for (size_t i = key_frame + 1, i_end = end_frame;
        i <= i_end; ++ i)
    {
        std::cout << "tracking [frame " << i << "]" << std::endl;

        // EM + ARAP tracking 
        PointCloud* forward_cloud = points_file_system->getPointCloud(i);
        tracking_system_->ea_registration(*forward_cloud, *forward_flower);
        forward_flower->save(flowers_folder, i);
        forward_flower->update();

        points_file_system->hidePointCloud(i - 1);
        points_file_system->showPointCloud(i);

    }
    forward_flower->hide();

    //std::cout << "Backward Tracking..." << std::endl;
    //backward_flower->show();
    //for (int i = key_frame, i_end = start_frame;
    //    i >= i_end; -- i)
    //{
    //    std::cout << "tracking [frame " << i << "]" << std::endl;

    //    // EM + ARAP tracking 
    //    PointCloud* backward_cloud = points_file_system->getPointCloud(i);
    //    tracking_system_->ea_registration(*backward_cloud, *backward_flower);
    //    backward_flower->save(flowers_folder, i);
    //    backward_flower->update();

    //    points_file_system->hidePointCloud(i + 1);
    //    points_file_system->showPointCloud(i);

    //}
    //backward_flower->hide();

    std::cout << "EM + ARAP Tracking Finished!" << std::endl;
}


WEATrackThread::WEATrackThread(TrackingSystem* tracking_system)
    :QThread()
{
    tracking_system_ = tracking_system;
}

WEATrackThread::~WEATrackThread()
{}

void WEATrackThread::run()
{
    std::cout << "WEM + ARAP Tracking Starts..." << std::endl;

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

    Flower* forward_flower = new Flower(*flower);
    osg::ref_ptr<PointCloud> aligned_cloud = points_file_system->getPointCloud(key_frame);
    aligned_cloud->flower_segmentation(forward_flower);
    osg::ref_ptr<PointCloud> forward_cloud;

    std::string workspace = MainWindow::getInstance()->getWorkspace();
    QDir flowers_dir(QString(workspace.c_str()));
    flowers_dir.mkdir("flowers");
    std::string flowers_folder = flowers_dir.absolutePath().toStdString() + "/flowers";


    std::cout << "Forward Tracking..." << std::endl;
    forward_flower->show();

    // WEM + ARAP tracking 
    for (size_t i = key_frame + 1, i_end = end_frame;
        i <= i_end; ++ i)
    {
        std::cout << "tracking [frame " << i << "]" << std::endl;
     
        forward_flower->determineWeights(aligned_cloud);  // weights of gmm based on aligned cloud
        //forward_flower->determineVisibility(); // visibility of vertices

        forward_cloud = points_file_system->getPointCloud(i);
        forward_cloud->flower_segmentation(forward_flower);

        tracking_system_->wea_registration(*forward_cloud, *forward_flower);
        forward_flower->save(flowers_folder, i);
        forward_flower->update();

        points_file_system->hidePointCloud(i - 1);
        points_file_system->showPointCloud(i);
        aligned_cloud = forward_cloud;
    }
    forward_flower->hide();

    std::cout << "WEM + ARAP Tracking Finished!" << std::endl;
}


//////////////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////////////
EETrackThread::EETrackThread( TrackingSystem* tracking_system )
	:QThread()
{
	tracking_system_ = tracking_system;
}

EETrackThread::~EETrackThread()
{

}

void EETrackThread::run()
{
	std::cout << "EM + Elastic Tracking Starts..." << std::endl;

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

	Flower* forward_flower = new Flower(*flower);
	Flower* backward_flower = new Flower(*flower);

	std::string workspace = MainWindow::getInstance()->getWorkspace();
	QDir flowers_dir(QString(workspace.c_str()));
	flowers_dir.mkdir("flowers-2");
	std::string flowers_folder = flowers_dir.absolutePath().toStdString() + "/flowers-2";


	std::cout << "Forward Tracking..." << std::endl;
	forward_flower->show();
	for (size_t i = key_frame + 1, i_end = end_frame;
		i <= i_end; ++ i)
	{
		std::cout << "tracking [frame " << i << "]" << std::endl;

		// EM + ARAP tracking 
		PointCloud* forward_cloud = points_file_system->getPointCloud(i);
		tracking_system_->ee_registration(*forward_cloud, *forward_flower);
		forward_flower->save(flowers_folder, i);
		forward_flower->update();

		points_file_system->hidePointCloud(i - 1);
		points_file_system->showPointCloud(i);

	}
	forward_flower->hide();

	std::cout << "Backward Tracking..." << std::endl;
	backward_flower->show();
	for (int i = key_frame, i_end = start_frame;
		i >= i_end; -- i)
	{
		std::cout << "tracking [frame " << i << "]" << std::endl;

		// EM + ARAP tracking 
		PointCloud* backward_cloud = points_file_system->getPointCloud(i);
		tracking_system_->ee_registration(*backward_cloud, *backward_flower);
		backward_flower->save(flowers_folder, i);
		backward_flower->update();

		points_file_system->hidePointCloud(i + 1);
		points_file_system->showPointCloud(i);

	}
	//   forward_flower->hide();




	std::cout << "EM + Elastic Tracking Finished!" << std::endl;

}
