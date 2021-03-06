
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
#include "tip_detector.h"
#include "collision_detector.h"
#include "trajectory_model.h"
#include "solver.h"
#include "fitting_error_measurer.h"



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


LATrackThread::LATrackThread(TrackingSystem* tracking_system)
    :QThread()
{
    tracking_system_ = tracking_system;
}

LATrackThread::~LATrackThread()
{}

void LATrackThread::run()
{
    std::cout << "LBS + ARAP Tracking Starts..." << std::endl;

    int key_frame = MainWindow::getInstance()->getParameters()->getKeyFrame();
    int start_frame = MainWindow::getInstance()->getParameters()->getStartFrame();
    int end_frame = MainWindow::getInstance()->getParameters()->getEndFrame();

    PointsFileSystem* points_file_system = tracking_system_->getPointsFileSystem();
    MeshFileSystem* mesh_file_system = tracking_system_->getMeshFileSystem();

    /*int start_frame = points_file_system->getStartFrame();
    int end_frame = points_file_system->getEndFrame();*/

    QSet<QPersistentModelIndex> mesh_indexes = mesh_file_system->getCheckedIndexes();

    // build flower structure, memory leak...
    Flower* flower = new Flower;
    for (size_t i = 0, i_end = mesh_indexes.size(); i < i_end; ++ i)
    {
        osg::ref_ptr<Petal> petal_template = mesh_file_system->getMeshModel(mesh_indexes.values().at(i));
        flower->getPetals().push_back(*petal_template); // deep copy
        mesh_file_system->hideMeshModel(mesh_indexes.values().at(i));
    }
    flower->reorder();

    std::string workspace = MainWindow::getInstance()->getWorkspace();
    QDir flowers_dir(QString(workspace.c_str()));
    flowers_dir.mkdir("flowers");
    std::string flowers_folder = flowers_dir.absolutePath().toStdString() + "/flowers";

    TipDetector tip_detector;
    int tip_bin_num = MainWindow::getInstance()->getParameters()->getBinNum();
    float tip_knn_radius = MainWindow::getInstance()->getParameters()->getKnnRadius();

    /*osg::ref_ptr<TrajectoryModel> traj_model = new TrajectoryModel;
    traj_model->init(flower);
    traj_model->show();*/


    flower->show();
    std::cout << "tracking [frame " << key_frame << "]" << std::endl;

    osg::ref_ptr<PointCloud> key_cloud = points_file_system->getPointCloud(key_frame);
    key_cloud->setClosureCloud(points_file_system->getClosureCloudFilename(key_frame));

    std::cout << "detect flower boundary" << std::endl;
    tip_detector.setFlower(flower);
    tip_detector.detectBoundary(tip_bin_num, tip_knn_radius);

    std::cout << "detect point cloud boundary" << std::endl;
    tip_detector.setPointCloud(key_cloud);
    tip_detector.detectBoundary(tip_bin_num, tip_knn_radius);

    std::cout << "flower segmentation" << std::endl;
    key_cloud->fitting_region(flower, nullptr);

    tracking_system_->la_registration(*key_cloud, *flower, key_frame);
    flower->save(flowers_folder, key_frame);
    flower->hide();

    /*for (int i = 0; i < flower->getPetals().size(); ++ i)
    {
    flower->getPetals()[i].getObjName() = QString("%1.obj").arg(i).toStdString();
    }*/

    Flower* forward_flower = new Flower(*flower);
    osg::ref_ptr<PointCloud> forward_cloud;

    std::cout << "Forward Tracking..." << std::endl;
    forward_flower->show();

    // LBS + ARAP tracking 
    for (size_t i = key_frame+1, i_end = end_frame;
        i <= i_end; ++ i)
    {
        std::cout << "tracking [frame " << i << "]" << std::endl;

        forward_cloud = points_file_system->getPointCloud(i);
        forward_cloud->setClosureCloud(points_file_system->getClosureCloudFilename(i));

        std::cout << "detect flower boundary" << std::endl;
        tip_detector.setFlower(forward_flower);
        tip_detector.detectBoundary(tip_bin_num, tip_knn_radius);

        std::cout << "detect point cloud boundary" << std::endl;
        tip_detector.setPointCloud(forward_cloud);
        tip_detector.detectBoundary(tip_bin_num, tip_knn_radius);

        std::cout << "flower segmentation" << std::endl;
        forward_cloud->fitting_region(forward_flower, nullptr);

        forward_flower->setTransFolder(flowers_folder, i-1);
        Solver::is_forward_ = true;

        tracking_system_->la_registration(*forward_cloud, *forward_flower, i);
        forward_flower->save(flowers_folder, i);
        forward_flower->update();

        points_file_system->hidePointCloud(i - 1);
        points_file_system->showPointCloud(i);

    }
    forward_flower->hide();

    Flower* backward_flower = new Flower(*flower);
    osg::ref_ptr<PointCloud> backward_cloud;

    std::cout << "Backward Tracking..." << std::endl;
    backward_flower->show();
    for (int i = key_frame-1, i_end = start_frame;
        i >= i_end; -- i)
    {
        std::cout << "tracking [frame " << i << "]" << std::endl;
        backward_cloud = points_file_system->getPointCloud(i);
        backward_cloud->setClosureCloud(points_file_system->getClosureCloudFilename(i));

        std::cout << "detect flower boundary" << std::endl;
        tip_detector.setFlower(backward_flower);
        tip_detector.detectBoundary(tip_bin_num, tip_knn_radius);

        std::cout << "detect point cloud boundary" << std::endl;
        tip_detector.setPointCloud(backward_cloud);
        tip_detector.detectBoundary(tip_bin_num, tip_knn_radius);

        std::cout << "flower segmentation" << std::endl;
        backward_cloud->fitting_region(backward_flower, nullptr);

        backward_flower->setTransFolder(flowers_folder, i+1);
        Solver::is_forward_ = false;

        tracking_system_->la_registration(*backward_cloud, *backward_flower, i);
        backward_flower->save(flowers_folder, i);
        backward_flower->update();

        points_file_system->hidePointCloud(i + 1);
        points_file_system->showPointCloud(i);

    }

    backward_flower->hide();

    std::cout << "LBS + ARAP Tracking Finished!" << std::endl;
}


MotionTransferThread::MotionTransferThread(TrackingSystem* tracking_system)
    :QThread()
{
    tracking_system_ = tracking_system;
}

MotionTransferThread::~MotionTransferThread()
{}

void MotionTransferThread::run()
{
    std::cout << "Motion Transfer Starts..." << std::endl;

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
    flower->reorder();

    std::string workspace = MainWindow::getInstance()->getWorkspace();
    QDir flowers_dir(QString(workspace.c_str()));
    flowers_dir.mkdir("transfered_flowers");
    std::string flowers_folder = flowers_dir.absolutePath().toStdString() + "/transfered_flowers";

    flower->show();
    osg::ref_ptr<PointCloud> key_cloud = points_file_system->getPointCloud(key_frame);
    key_cloud->setClosureCloud(points_file_system->getClosureCloudFilename(key_frame));
    tracking_system_->mt_registration(*key_cloud, *flower, key_frame);
    flower->save(flowers_folder, key_frame);
    flower->update();
    flower->hide();

    Flower* forward_flower = new Flower(*flower);
    Flower* backward_flower = new Flower(*flower);

    /*std::cout << "Forward Tracking..." << std::endl;
    osg::ref_ptr<PointCloud> forward_cloud;
    forward_flower->show();
    for (size_t i = key_frame+1, i_end = end_frame;
    i <= i_end; ++ i)
    {
    std::cout << "tracking [frame " << i << "]" << std::endl;

    forward_cloud = points_file_system->getPointCloud(i);
    forward_cloud->setClosureCloud(points_file_system->getClosureCloudFilename(i));

    tracking_system_->mt_registration(*forward_cloud,*forward_flower, i);
    forward_flower->save(flowers_folder, i);
    forward_flower->update();
    }
    forward_flower->hide();*/

    std::cout << "Backward Tracking..." << std::endl;
    osg::ref_ptr<PointCloud> backward_cloud;
    backward_flower->show();
    for (int i = key_frame-1, i_end = start_frame;
        i >= i_end; -- i)
    {
        std::cout << "tracking [frame " << i << "]" << std::endl;

        backward_cloud = points_file_system->getPointCloud(i);
        backward_cloud->setClosureCloud(points_file_system->getClosureCloudFilename(i));

        tracking_system_->mt_registration(*backward_cloud, *backward_flower, i);
        backward_flower->save(flowers_folder, i);
        backward_flower->update();
    }
    backward_flower->hide();

    std::cout << "Motion Transfer Finished!" << std::endl;
}


TipThread::TipThread(TrackingSystem* tracking_system)
    :QThread()
{
    tracking_system_ = tracking_system;
}

TipThread::~TipThread()
{}

void TipThread::run()
{
    std::cout << "Detecting Tips of Point Clouds..." << std::endl;

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

    TipDetector tip_detector;

    /*for (int i = start_frame; i <= end_frame; ++ i)
    {
    std::cout << "frame " << i << std::endl;

    osg::ref_ptr<PointCloud> cloud = points_file_system->getPointCloud(i);

    tip_detector.setPointCloud(cloud);
    tip_detector.detectTips(12, 12);

    points_file_system->hidePointCloud(i - 1);
    points_file_system->showPointCloud(i);
    cloud->expire();
    }*/
    flower->show();
    tip_detector.setFlower(flower);
    tip_detector.detectBoundary(12, 12);
    flower->update();

    /*osg::ref_ptr<PointCloud> cloud = points_file_system->getPointCloud(32);
    tip_detector.setPointCloud(cloud);
    tip_detector.detectBoundary(12, 12);
    cloud->expire();*/


    std::cout << "Tips Detection Finished!" << std::endl;
}

BoundaryThread::BoundaryThread(TrackingSystem* tracking_system)
    :QThread()
{
    tracking_system_ = tracking_system;
}

BoundaryThread::~BoundaryThread()
{}

void BoundaryThread::run()
{
    std::cout << "Detecting Boundary of Point Clouds..." << std::endl;

    PointsFileSystem* points_file_system = tracking_system_->getPointsFileSystem();

    int start_frame = points_file_system->getStartFrame();
    int end_frame = points_file_system->getEndFrame();
    
    TipDetector tip_detector;
    for (int i = 93; i <= 100; ++ i)
    {
        std::cout << "frame " << i << std::endl;

        osg::ref_ptr<PointCloud> cloud = points_file_system->getPointCloud(i);

        tip_detector.setPointCloud(cloud);
        tip_detector.detectBoundary(12, 12);

        points_file_system->hidePointCloud(i - 1);
        points_file_system->showPointCloud(i);
        cloud->expire();
    }

    std::cout << "Boundary Detection Finished!" << std::endl;
}

FittingErrorThread::FittingErrorThread(PointsFileSystem* points_file_system, 
	FlowersViewer* flower_viewer)
	:QThread()
{
	points_file_system_  = points_file_system;
	flower_viewer_ = flower_viewer;
}


FittingErrorThread::~FittingErrorThread()
{

}

void FittingErrorThread::run()
{
	std::string file_name = flower_viewer_->getFlowerFolder () + "/fitting_err.txt";
	std::cout << "Measure fitting error starts..." << std::endl;

	int start_frame = flower_viewer_->getStartFrame();
	int end_frame = flower_viewer_->getEndFrame();

	FittingErrorMeasurer error_measurer;
	points_file_system_->navigateToAFrame(start_frame);
	flower_viewer_->getFlower(start_frame);
	flower_viewer_->update();

	Eigen::VectorXd fitting_err(end_frame - start_frame + 1);

	for (size_t i = start_frame , i_end = end_frame;
		i <= i_end; ++i)
	{
		osg::ref_ptr<PointCloud> cloud = points_file_system_->getPointCloud(i);
		Flower& flower = flower_viewer_->getCurrentFlower();

		error_measurer.setFlowerAndPointCloud(&flower, cloud);
		error_measurer.computeDisFromCloud2Flower();
		fitting_err[i-start_frame] = error_measurer.getMeanDis();

		points_file_system_->navigateToNextFrame();
		flower_viewer_->next();
		flower_viewer_->update();
	}
	// write the results
	std::cout << "Measure fitting error ends!" << std::endl;

	std::ofstream out(file_name.c_str());
	out << fitting_err << std::endl;
	out.close();
}

RegionMatchingThread::RegionMatchingThread(PointCloud* point_cloud, Flower* flower)
{
    point_cloud_ = point_cloud;
    flower_ = flower;
}

RegionMatchingThread::~RegionMatchingThread()
{

}

void RegionMatchingThread::run()
{
    point_cloud_->region_matching(flower_);
}




CollisionDetectionThread::CollisionDetectionThread( Flower* flower )
{
	flower_ = flower;
}

CollisionDetectionThread::~CollisionDetectionThread()
{

}

void CollisionDetectionThread::run()
{
	CollisionDetector collision_detector;
	collision_detector.setFlower( flower_ );
	collision_detector.checkCollision();
	collision_detector.resolveCollision();
	flower_->update();
}
