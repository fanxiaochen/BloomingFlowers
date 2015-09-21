
#include <QToolTip>
#include <QKeyEvent>
#include <QSettings>
#include <QGridLayout>
#include <QDockWidget>
#include <QFileDialog>
#include <QMessageBox>
#include <QGroupBox>
#include <QCheckBox>
#include <QApplication>
#include <QInputDialog>
#include <QTextStream>

#include <osgDB/ReadFile>

#include "main_window.h"
#include "file_viewer_widget.h"
#include "scene_widget.h"
#include "points_file_system.h"
#include "mesh_file_system.h"
#include "tracking_system.h"
#include "parameters.h"
#include "registrator.h"
#include "flower.h"
#include "task_thread.h"
#include "trajectory_model.h"
#include "transfer.h"
#include "screen_capture.h"


MainWindow::MainWindow(void)
    :points_path_("."),
    mesh_path_("."),
    points_files_(NULL),
    mesh_files_(NULL),
    points_widget_(NULL),
    mesh_widget_(NULL),
    tracking_system_(NULL),
    parameters_(NULL),
    registrator_(NULL),
    flowers_viewer_(NULL),
    trajectory_model_(NULL)
{
    ui_.setupUi(this);

    MainWindowInstancer::getInstance().main_window_ = this;

    init();
}

MainWindow::~MainWindow()
{
    saveSettings();

    delete points_files_;
    delete mesh_files_;
    delete points_widget_;
    delete mesh_widget_;
    delete scene_widget_;
    delete tracking_system_;
    delete parameters_;
    delete registrator_;
    delete flowers_viewer_;
    delete trajectory_model_;
    return;
}

void MainWindow::slotShowInformation(const QString& information)
{
    QToolTip::showText(QCursor::pos(), information);
}

void MainWindow::showInformation(const std::string& information)
{
    emit showInformationRequested(information.c_str());
}

void MainWindow::slotShowStatus(const QString& status, int timeout)
{
     ui_.statusBar->showMessage(status, timeout);
}

void MainWindow::showStatus(const std::string& status, int timeout)
{
    emit showStatusRequested(status.c_str(), timeout);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);

    return;
}

void MainWindow::keyPressEvent(QKeyEvent* event)
{
    PointsFileSystem* points_file = dynamic_cast<PointsFileSystem*>(points_files_);

    switch(event->key())
    {
	case(Qt::Key_Up):
		if( points_files_ )
		{
			points_file->navigateToPreviousFrame();
		}
		break;
	case(Qt::Key_Down):
		if( points_file )
		{
			points_file->navigateToNextFrame();
		}	
		break;
	case(Qt::Key_Left):
		if( flowers_viewer_ )
		{
			flowers_viewer_->previous();
			flowers_viewer_->update();
		}		
		break;
	case(Qt::Key_Right):
		if( flowers_viewer_ )
		{
			flowers_viewer_->next();
			flowers_viewer_->update();
		}
		break;
	case(Qt::Key_PageUp):
		if( points_file )
		{
			points_file->navigateToPreviousFrame();
		}
		if( flowers_viewer_ )
		{
			flowers_viewer_->previous();
			flowers_viewer_->update();
		}
		break;
	case (Qt::Key_PageDown):
		if( points_file )
		{
			points_file->navigateToNextFrame();
		}
		if( flowers_viewer_ )
		{
			flowers_viewer_->next();
			flowers_viewer_->update();
		}
		break;
	default:
		QMainWindow::keyPressEvent(event);
		break;
    

    }

    return;
}

MainWindow* MainWindow::getInstance()
{
    assert(MainWindowInstancer::getInstance().main_window_ != NULL);
    return MainWindowInstancer::getInstance().main_window_;
}

void MainWindow::init(void)
{

    points_files_ = new PointsFileSystem;
    mesh_files_ = new MeshFileSystem;
    points_widget_ = new FileViewerWidget(this, points_files_);
    mesh_widget_ = new FileViewerWidget(this, mesh_files_);

    tracking_system_ = new TrackingSystem(
        dynamic_cast<PointsFileSystem*>(points_files_), dynamic_cast<MeshFileSystem*>(mesh_files_));

    parameters_ = new Parameters;
    registrator_ = new Registrator;

    QDockWidget* points_viewer = new QDockWidget("Points Viewer", this);
    addDockWidget(Qt::LeftDockWidgetArea, points_viewer);
    points_viewer->setAllowedAreas(Qt::LeftDockWidgetArea|Qt::RightDockWidgetArea);

    points_widget_->setParent(points_viewer);
    points_viewer->setWidget(points_widget_);

    QDockWidget* mesh_viewer = new QDockWidget("Mesh Viewer", this);
    addDockWidget(Qt::LeftDockWidgetArea, mesh_viewer);
    mesh_viewer->setAllowedAreas(Qt::LeftDockWidgetArea|Qt::RightDockWidgetArea);

    mesh_widget_->setParent(mesh_viewer);
    mesh_viewer->setWidget(mesh_widget_);

    setRenderingBox();

    scene_widget_ = new SceneWidget(this);
    setCentralWidget(scene_widget_);
    scene_widget_->startRendering();

    connect(this, SIGNAL(showInformationRequested(const QString&)), this, SLOT(slotShowInformation(const QString&)));
    connect(this, SIGNAL(showStatusRequested(const QString&, int)), this, SLOT(slotShowStatus(const QString&, int)));



	loadSettings();
	points_widget_->setWorkspace( QString(points_path_.c_str()));
	mesh_widget_->setWorkspace( QString(mesh_path_.c_str() ));
	

    connect(ui_.actionLoadPoints, SIGNAL(triggered()), this, SLOT(slotLoadPoints()));
    connect(ui_.actionLoadMesh, SIGNAL(triggered()), this, SLOT(slotLoadMesh()));
    connect(ui_.actionLoadParameters, SIGNAL(triggered()), this, SLOT(slotLoadParameters()));
    connect(ui_.actionSaveParameters, SIGNAL(triggered()), this, SLOT(slotSaveParameters()));
    connect(ui_.actionLoadAxis, SIGNAL(triggered()), this, SLOT(slotLoadAxis()));
    connect(ui_.actionLoadFlowers, SIGNAL(triggered()), this, SLOT(slotLoadFlowers()));
	connect(ui_.actionLoadCamera, SIGNAL( triggered () ),this, SLOT( loadCamera()) );
	connect(ui_.actionSaveCamera, SIGNAL( triggered () ),this, SLOT( saveCamera()) );
    connect(ui_.actionSnapshotAllFrames, SIGNAL(triggered()), this, SLOT(slotSnapshotAllFrames()));

    connect(ui_.actionEMARAP, SIGNAL(triggered()), tracking_system_, SLOT(em_arap()));
    connect(ui_.actionWEMARAP, SIGNAL(triggered()), tracking_system_, SLOT(wem_arap()));
	connect(ui_.actionEMElastic, SIGNAL(triggered()), tracking_system_, SLOT(em_elastic()));
    connect(ui_.actionLBSARAP, SIGNAL(triggered()), tracking_system_, SLOT(lbs_arap()));
	connect(ui_.actionLights, SIGNAL(triggered()), scene_widget_, SLOT(setLight()));

    connect(ui_.actionTipDetection, SIGNAL(triggered()), tracking_system_, SLOT(detectTip()));
    connect(ui_.actionBoundaryDetection, SIGNAL(triggered()), tracking_system_, SLOT(detectBoundary()));
    connect(ui_.actionRegionProbability, SIGNAL(triggered()), this, SLOT(region_probability()));
	connect(ui_.actionCollision_Detection, SIGNAL(triggered()), this, SLOT(collision_detection()));
    connect(ui_.actionTrajectoriesGeneration, SIGNAL(triggered()), this, SLOT(trajectories_generation()));
    connect(ui_.actionMergePetals, SIGNAL(triggered()), this, SLOT(merge_petals()));
    connect(ui_.actionPetalSequences, SIGNAL(triggered()), this, SLOT(petal_sequences()));
	connect(ui_.actionSavePlys, SIGNAL(triggered()), this, SLOT(save_plys()));
    connect(ui_.actionCameraViews, SIGNAL(triggered()), this, SLOT(camera_views()));
	connect(ui_.actionSequenceSmoothing, SIGNAL(triggered()), this, SLOT(sequence_smoothing()));

    connect(ui_.actionTransfer, SIGNAL(triggered()), this, SLOT(transfer()));
    connect(ui_.actionMultiLayer, SIGNAL(triggered()), this, SLOT(multi_layer()));
    connect(ui_.actionInterpolation, SIGNAL(triggered()), this, SLOT(interpolation()));
    // connect


    return;
}

void MainWindow::setRenderingBox()
{
    //CheckBox
    std::vector<std::pair<QString,bool>> render_names;

    render_names.push_back( std::make_pair("Show Texture", false));
    render_names.push_back( std::make_pair("Show Skeleton", false));
    render_names.push_back( std::make_pair("Show Trajectory", false));
    render_names.push_back( std::make_pair("Show Boundary", false));
    render_names.push_back( std::make_pair("Show Tips", false));
    render_names.push_back( std::make_pair("Open Lights", false));

    QGroupBox* rendering_group = new QGroupBox(tr("Rendering Box"));
    QVBoxLayout *layout = new QVBoxLayout();

    for( int i = 0 ;i!= render_names.size(); ++i)
    {
        QCheckBox* render_box(new QCheckBox(render_names[i].first, this));
        render_box->setChecked( render_names[i].second );
        layout->addWidget(render_box);
        Check_list_.push_back(render_box);
        connect(render_box,SIGNAL(toggled(bool)), this, SLOT(slotSendCheckBoxRenderState(void)));
    }

    rendering_group->setLayout(layout);
    ui_.toolBar->addWidget(rendering_group);

}

bool MainWindow::slotSendCheckBoxRenderState()
{
    MeshFileSystem* mesh_file_system = dynamic_cast<MeshFileSystem*>(mesh_files_);
    PointsFileSystem* points_file_system = dynamic_cast<PointsFileSystem*>(points_files_);


	std::map<std::string, bool> states;
    int num = Check_list_.size();
    for( size_t i = 0; i != num; ++i )
    {
        bool flag = Check_list_[i]->isChecked();
        std::string name = Check_list_[i]->text().toStdString();

		states[name] = flag;
	}

	// for mesh viewer
	{
		QSet<QPersistentModelIndex> mesh_indexes = mesh_file_system->getCheckedIndexes();
		for (size_t i = 0, i_end = mesh_indexes.size(); i < i_end; ++ i)
		{
			osg::ref_ptr<MeshModel> mesh_model = mesh_file_system->getMeshModel(mesh_indexes.values().at(i));
			mesh_model->getShowTexture() = states["Show Texture"];
			if (!mesh_model->getSkeleton()->isEmpty())
			{
				mesh_model->showSkeletonState(states["Show Skeleton"]);
			}
			mesh_model->expire();
		}
	}
	

	// for flower viewer
	if (flowers_viewer_ != NULL)
	{
		Flower& current_flower = flowers_viewer_->getCurrentFlower();
		current_flower.setTextureState(states["Show Texture"]);
		current_flower.setSkeletonState(states["Show Skeleton"]);
		current_flower.update();
	}

	// for points
	{
		PointsFileSystem* points_file_system = tracking_system_->getPointsFileSystem();

		int start_frame = points_file_system->getStartFrame();
		int end_frame = points_file_system->getEndFrame();


		for (size_t i = start_frame, i_end = end_frame; i <= i_end; ++ i)
		{
			osg::ref_ptr<PointCloud> point_cloud = points_file_system->getPointCloud(i);
			point_cloud->getShowBoundary() = states["Show Boundary"];
			point_cloud->getShowTips() = states["Show Tips"];
			point_cloud->expire();
		}
	}

	// trajectory mode
	if( trajectory_model_ )
	{
		trajectory_model_->showState( states["Show Trajectory"] );
	}
// 	// 重设光源
//     scene_widget_->setLight();  
	
    return true;
}

// only one flower and point cloud needed
bool MainWindow::region_probability()
{
    MeshFileSystem* mesh_file_system = dynamic_cast<MeshFileSystem*>(mesh_files_);
    PointsFileSystem* points_file_system = dynamic_cast<PointsFileSystem*>(points_files_);

    QSet<QPersistentModelIndex> mesh_indexes = mesh_file_system->getCheckedIndexes();
    // build flower structure, memory leak...
    Flower* flower = new Flower;
    for (size_t i = 0, i_end = mesh_indexes.size(); i < i_end; ++ i)
    {
        osg::ref_ptr<Petal> petal_template = mesh_file_system->getMeshModel(mesh_indexes.values().at(i));
        flower->getPetals().push_back(*petal_template); // deep copy
        mesh_file_system->hideMeshModel(mesh_indexes.values().at(i));
    }
  
    QSet<QPersistentModelIndex> points_indexes = points_file_system->getCheckedIndexes();
    osg::ref_ptr<PointCloud> point_cloud = points_file_system->getPointCloud(points_indexes.values().at(0));

    RegionMatchingThread* rm_thread = new RegionMatchingThread(point_cloud, flower);
    connect(rm_thread, SIGNAL(finished()), rm_thread, SLOT(quit()));
    rm_thread->start();

    return true;
}


bool MainWindow::merge_petals()
{
    QString flower_folder = QFileDialog::getExistingDirectory(this, tr("Load Flowers"), workspace_.c_str(), QFileDialog::ShowDirsOnly);

    if (flower_folder.isEmpty())
        return false;

    flowers_viewer_ = new FlowersViewer(flower_folder.toStdString());
    flowers_viewer_->computeFrameRange();
    
    ObjWriter obj_writer;
    obj_writer.merge(flowers_viewer_);
    return true;
}

bool MainWindow::petal_sequences()
{
    QString flower_folder = QFileDialog::getExistingDirectory(this, tr("Load Flowers"), workspace_.c_str(), QFileDialog::ShowDirsOnly);

    if (flower_folder.isEmpty())
        return false;

    flowers_viewer_ = new FlowersViewer(flower_folder.toStdString());
    flowers_viewer_->computeFrameRange();

    int start_frame = flowers_viewer_->getStartFrame();
    int end_frame = flowers_viewer_->getEndFrame();

    std::cout << start_frame << end_frame << std::endl;

    QDir flowers_dir(flower_folder);
    flowers_dir.mkdir("petal sequences");
    std::string petals_folder = flowers_dir.absolutePath().toStdString() + "/petal sequences";
    
    int petal_num = MainWindow::getInstance()->getParameters()->getPetalNum();
    for (size_t j = 0; j < petal_num; ++ j)
    {
        QDir petals_dir(QString(petals_folder.c_str()));
        petals_dir.mkdir(QString("petal-%1").arg(j));
    }
    
    ObjWriter obj_writer;

    for (size_t i = start_frame; i <= end_frame; ++ i)
    {
        obj_writer.copyPetals(flower_folder.toStdString(), i, petals_folder);
    }

    std::cout << "petal sequences finished" << std::endl;

    return true;
}

// overwrite basic flower sequence
bool MainWindow::sequence_smoothing()
{
    int start_frame = flowers_viewer_->getStartFrame();
    int end_frame = flowers_viewer_->getEndFrame();

    std::string flower_folder = flowers_viewer_->getFlowerFolder();

    float sigma = 0.5;

    for (int i = start_frame; i <= end_frame; ++ i)
    {
        osg::ref_ptr<Flower> f_i = flowers_viewer_->flower(i);
        if (i == start_frame || i == end_frame)
            continue;
        else
        {
            osg::ref_ptr<Flower> _f_i = flowers_viewer_->flower(i-1);
            osg::ref_ptr<Flower> f_i_ = flowers_viewer_->flower(i+1);

            for (int j = 0, j_end = f_i->getPetals().size(); j < j_end; ++ j)
            {
                Petal& p_i_j = f_i->getPetals()[j];
                Petal& _p_i_j = _f_i->getPetals()[j];
                Petal& p_i_j_ = f_i_->getPetals()[j];

                for (int k = 0, k_end = p_i_j.getVertices()->size(); k < k_end; ++ k)
                {
                    osg::Vec3& p_i_j_k = p_i_j.getVertices()->at(k);
                    osg::Vec3& _p_i_j_k = _p_i_j.getVertices()->at(k);
                    osg::Vec3& p_i_j_k_ = p_i_j_.getVertices()->at(k);

                    p_i_j_k += (_p_i_j_k-p_i_j_k*2+p_i_j_k_)/4.0 * exp(-((p_i_j_k-_p_i_j_k).length2() + (p_i_j_k-p_i_j_k_).length2()) / (sigma*sigma));
                }
            }
         }
        f_i->save(flower_folder, i);
    }

    std::cout << "sequence smoothing finished!" << std::endl;

    return true;
}


bool MainWindow::save_plys()
{
    PointsFileSystem* points_file = dynamic_cast<PointsFileSystem*>(points_files_);
    int start_frame = points_file->getStartFrame();
    int end_frame = points_file->getEndFrame();
    
    for (int i = start_frame; i <= end_frame; ++ i)
    {
        points_file->savePointCloudAsPly(i);
    }

	return true;
}


bool MainWindow::camera_views()
{
	// default, six views
	const int view_num = 6;
	double angle = (2 * M_PI) / view_num;
	
// 
// 	std::string output_file = workspace_ + "\\world2cameraT.txt";
// 	std::ofstream out(output_file);
// 	for( int i = 0 ;i!= view_num; ++i )
// 	{
// 		if( i== 1)
// 			continue;
// 		// the first camera:
// 		osg::Vec3d eye0(0,0,0);
// 		osg::Vec3d zDir0(0,0,1);   // center - eye
// 		osg::Vec3d yDir0(0,-1,0);   // up
// 		osg::Vec3d xDir0(1,0,0);    
// 
// 		osg::Matrix rotation = registrator_->getRotationMatrix(-i * angle);
// 		osg::Vec3d eyeC = rotation.preMult(eye0); 
// 		osg::Vec3d zDirC = rotation.preMult(eye0+zDir0)-eyeC;
// 		osg::Vec3d yDirC = rotation.preMult(eye0+yDir0)-eyeC;
// 		osg::Vec3d xDirC = rotation.preMult(eye0+xDir0)-eyeC;
// 
// 		osg::Vec3d t(-xDirC*eyeC, -yDirC*eyeC, -zDirC*eyeC);
// 		out << xDirC.x() << " " << xDirC.y() << " " << xDirC.z() << " ";
// 		out << yDirC.x() << " " << yDirC.y() << " " << yDirC.z() << " ";
// 		out << zDirC.x() << " " << zDirC.y() << " " << zDirC.z() << " ";
// 		out << t.x() << " " << t.y() << " " << t.z()  << std::endl;
// 
// // 		osg::Vec3d t(-xDirC*eyeC, -yDirC*eyeC, -zDirC*eyeC);
// // 		out << xDirC.x() << " " << xDirC.y() << " " << xDirC.z() << " " << t.x() << std::endl;
// // 		out << yDirC.x() << " " << yDirC.y() << " " << yDirC.z() << " " << t.y() << std::endl;
// // 		out << zDirC.x() << " " << zDirC.y() << " " << zDirC.z() << " " << t.z() << std::endl;
// // 		out << 0 << " " << 0 << " " << 0 <<" " << 1 << std::endl;
// 	}
// 	out.close(;)

	bool ok;
	int view_idx  = QInputDialog::getInt(this, tr("QInputDialog::getInt()"),
		tr("View Index:"),  0, 0, 6, 1, &ok);
	if (ok )
	{
		osg::BoundingSphere bounding_sphere = scene_widget_->getBoundingSphere();

		// the first camera:
		osg::Vec3d eye0(0,0,0);
		osg::Vec3d zDir0(0,0,1);   // center - eye
		osg::Vec3d yDir0(0,-1,0);   // up
		osg::Vec3d xDir0(1,0,0);  

		osg::Matrix rotation = registrator_->getRotationMatrix(-view_idx * angle);
		osg::Vec3d eyeC = rotation.preMult(eye0);
		osg::Vec3d zDirC = rotation.preMult(eye0+zDir0)-eyeC;
		osg::Vec3d yDirC = rotation.preMult(eye0+yDir0)-eyeC;

		scene_widget_->getCamera()->setViewMatrixAsLookAt(eyeC, zDirC+eyeC, yDirC);

		osgGA::CameraManipulator* camera_manipulator = scene_widget_->getCameraManipulator();
		osg::Vec3d e, c, u;
		camera_manipulator->getHomePosition(e, c, u);

		camera_manipulator->setHomePosition(eyeC, zDirC+eyeC, yDirC);
		camera_manipulator->home(0);


// 		QFile txt_file(filename);
// 		txt_file.open(QIODevice::WriteOnly | QIODevice::Text);
// 		QTextStream txt_file_stream(&txt_file);
// 		txt_file_stream << eye.x() << " " << eye.y() << " " << eye.z() << "\n";
// 		txt_file_stream << center.x() << " " << center.y() << " " << center.z() << "\n";
// 		txt_file_stream << up.x() << " " << up.y() << " " << up.z() << "\n";

	}
	
    return true;
}


bool MainWindow::transfer()
{
    QString directory = QFileDialog::getExistingDirectory(this, tr("Transfer"), "transfer_flowers", QFileDialog::ShowDirsOnly);
    std::string transfer_folder = "D:/baidu disk/WorkSpace/Projects/BloomingFlower/BloomingFlowers/data/applications/transfers-lily";
    std::string transform_folder = transfer_folder + "/petal sequences";

    Transfer t(transform_folder);

    std::vector<int> order;
    order.push_back(3);
    order.push_back(4);
    order.push_back(5);

    std::string template_frame = transfer_folder + "/key frame";
    int key_frame = 32;
    int start_frame = 0;
    int end_frame = 127;

    t.loadFlower(template_frame, key_frame, order);

    t.setFlowerFolder(directory.toStdString());

    t.transfer(start_frame, end_frame);
   // t.transfer(false);
    std::cout << "transfer finished!" << std::endl;
    return true;
}

bool MainWindow::multi_layer()
{
    QString new_flower_folder = QFileDialog::getExistingDirectory(this, tr("flowers folder"), "flowers folder", QFileDialog::ShowDirsOnly);
    int time_interval = 10;

    int start_frame = flowers_viewer_->getStartFrame();
    int end_frame = flowers_viewer_->getEndFrame();

    for (size_t i = start_frame; i <= end_frame; ++ i)
    {
       if ((i - start_frame) < time_interval)
       {
           osg::ref_ptr<Flower> flower = flowers_viewer_->flower(start_frame);
           flower->save(new_flower_folder.toStdString(), i);
       }
       else 
       {
           osg::ref_ptr<Flower> flower = flowers_viewer_->flower(i-time_interval+1);
           flower->save(new_flower_folder.toStdString(), i);
       }
    }

    std::cout << "Dacheng Algorithm!" << std::endl;
    return true;
}

bool MainWindow::interpolation()
{
    QString new_flower_folder = QFileDialog::getExistingDirectory(this, tr("flowers folder"), "flowers folder", QFileDialog::ShowDirsOnly);
    TrajectoryModel traj_model;
    traj_model.interpolate(flowers_viewer_, new_flower_folder.toStdString());

    std::cout << "interpolation finished!" << std::endl;
    return true;
}


// only flower is needed
bool MainWindow::collision_detection()
{
	// build flower structure, memory leak...
	Flower* flower = new Flower;

	MeshFileSystem* mesh_file_system = dynamic_cast<MeshFileSystem*>(mesh_files_);
	// for mesh viewer
	QSet<QPersistentModelIndex> mesh_indexes = mesh_file_system->getCheckedIndexes();
	for (size_t i = 0, i_end = mesh_indexes.size(); i < i_end; ++ i)
	{
		osg::ref_ptr<Petal> petal_template = mesh_file_system->getMeshModel(mesh_indexes.values().at(i));
		flower->getPetals().push_back(*petal_template); // deep copy
	}
    flower->reorder();

	int petal_num = flower->getPetals().size();
	Eigen::MatrixXi petal_relation( petal_num, petal_num ); 
	// 0 means no relation between two petals; 
	// 1 means petal i should occludes petal j;
	// -1 means petal i should be occluded by petal j

// 	// lily
// 	petal_relation << 0,0,0,-1,0,-1,
// 		0, 0, 0, 0, -1, -1,
// 		0, 0, 0, -1, -1, 0,
// 		1, 0, 1, 0, 0 ,0,
// 		0, 1, 1, 0, 0, 0,
// 		1, 1, 0, 0, 0, 0;

	// orchid
	petal_relation << 
		0, 1, -1, -1, 0,
		-1, 0, -1, 0, -1,
		1, 1, 0, 0, 0,
		1, 0, 0, 0 ,0,
		0, 1, 0, 0, 0;
	// orchid
	flower->setPetalRelation( petal_relation);

	CollisionDetectionThread* cd_thread = new CollisionDetectionThread(flower);
	flower->show();
	connect(cd_thread, SIGNAL(finished()), cd_thread, SLOT(quit()));
	cd_thread->start();

	return true;
}

bool MainWindow::trajectories_generation()
{
    if (trajectory_model_ != nullptr)
        trajectory_model_->clear();

    trajectory_model_ = new TrajectoryModel;
    trajectory_model_->show();
    trajectory_model_->recoverFromFlowerViewer(flowers_viewer_);
    trajectory_model_->fittingAll();
    trajectory_model_->update();

    return true;
}


bool MainWindow::slotLoadPoints(void)
{
    QString directory = QFileDialog::getExistingDirectory(this, tr("Load Points"), points_path_.c_str(), QFileDialog::ShowDirsOnly);

    if (directory.isEmpty())
        return false;

    points_path_ = directory.toStdString();

    directory.resize(directory.size()-7); // remove string "/points", the workspace is based on points path
    workspace_ = directory.toStdString();

    // load parameters before load points
    QString para_file = QString(workspace_.c_str()) + "/parameters.xml";
    parameters_->load(para_file.toStdString());

    // load axis before load points
    QString axis_file = QString(workspace_.c_str()) + "/axis.txt";
    registrator_->load(axis_file);

    // load points
    scene_widget_->removeSceneChildren();
    points_widget_->setWorkspace(QString(points_path_.c_str()));
    

    return true;
}

bool MainWindow::slotLoadMesh(void)
{
    QString directory = QFileDialog::getExistingDirectory(this, tr("Load Mesh"), workspace_.c_str(), QFileDialog::ShowDirsOnly);

    if (directory.isEmpty())
        return false;

    mesh_path_ = directory.toStdString();

    mesh_widget_->setWorkspace(directory);

    return true;
}

bool MainWindow::slotLoadParameters()
{
    QString para_file = QFileDialog::getOpenFileName(this, tr("Load Parameters"), workspace_.c_str(),  tr("Files (*.xml)"));

    if (para_file.isEmpty())
        return false;

    return parameters_->load(para_file.toStdString());
}

bool MainWindow::slotSaveParameters()
{
    std::string para_file = workspace_ + "/parameters.xml";
    return parameters_->save(para_file);
}

bool MainWindow::slotLoadAxis()
{
    QString axis_file = QFileDialog::getOpenFileName(this, tr("Load Axis"), workspace_.c_str(),  tr("Registrator (*.txt)"));

    if (axis_file.isEmpty())
        return false;

    return registrator_->load(axis_file);
}

bool MainWindow::slotLoadFlowers()
{
    QString flower_folder = QFileDialog::getExistingDirectory(this, tr("Load Flowers"), workspace_.c_str(), QFileDialog::ShowDirsOnly);

    if (flower_folder.isEmpty())
        return false;

    flowers_viewer_ = new FlowersViewer(flower_folder.toStdString());
    flowers_viewer_->computeFrameRange();
    flowers_viewer_->getFlower();
    flowers_viewer_->show();
	slotSendCheckBoxRenderState();
	
    return true;

}

void MainWindow::loadSettings()
{
    QSettings settings("Blooming_Flower", "Blooming_Flower");

    points_path_ = settings.value("points_path").toString().toStdString();
	workspace_ = settings.value("workspace").toString().toStdString();
	mesh_path_ = settings.value("meshs_path").toString().toStdString();
    return;
}

void MainWindow::saveSettings()
{
    QSettings settings("Blooming_Flower", "Blooming_Flower");
    settings.setValue("points_path", QString(points_path_.c_str()));
	settings.setValue("workspace", QString(workspace_.c_str()));
	settings.setValue("meshs_path", QString( mesh_path_.c_str()) );

    return;
}


bool MainWindow::loadCamera()
{
	MainWindow* main_window = MainWindow::getInstance();
	QString filename = QFileDialog::getOpenFileName(this, "Load Camera",QString(workspace_.c_str()), "Camera (*.camera)");
	if (filename.isEmpty())
		return false;
	scene_widget_->readCameraParameters( filename );
	return true;
}


bool MainWindow::saveCamera()
{
	MainWindow* main_window = MainWindow::getInstance();
	QString filename = QFileDialog::getSaveFileName(this, "Save Camera",QString(workspace_.c_str()), "Camera (*.camera)");
	if (filename.isEmpty())
		return false;

	scene_widget_->writeCameraParameters( filename );
	return true;

}


bool MainWindow::slotShowYesNoMessageBox(const std::string& text, const std::string& informative_text)
{
    QMessageBox msg_box;
    msg_box.setText(text.c_str());
    msg_box.setInformativeText(informative_text.c_str());
    msg_box.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msg_box.setDefaultButton(QMessageBox::Yes);
    int ret = msg_box.exec();

    return (ret == QMessageBox::Yes);
}

bool MainWindow::slotSnapshotAllFrames()
{
	QString snapshot_folder = QFileDialog::getExistingDirectory(this, tr("Load Flowers"), workspace_.c_str(), QFileDialog::ShowDirsOnly);

	if (snapshot_folder.isEmpty())
		return false;

	PointsFileSystem* points_file = dynamic_cast<PointsFileSystem*>(points_files_);
	if( !points_file && !flowers_viewer_ )
		return false;

	int frame_num = 0;
	if( points_file) 
		frame_num = points_file->getEndFrame() - points_file->getCurrentFrame();
	else if( flowers_viewer_ )
	{
		frame_num = flowers_viewer_->getEndFrame() - points_file->getCurrentFrame();
	}

	std::string rootName = snapshot_folder.toStdString() + "/cap-";
	std::string extName = ".png";
	osg::ref_ptr< osgwTools::ScreenCapture > sc = new osgwTools::ScreenCapture(
		rootName, extName);

	osg::Camera*  camera = scene_widget_->getCamera();
	osg::Camera::DrawCallback* old = camera->getPostDrawCallback();
	camera->setPostDrawCallback( sc.get() );

	// capture current frames
	{
		sc->setCapture(true);
		sc->setUseFrameNumber(true);
		sc->setNumFramesToCapture(1);
		Sleep(500);
	}

	// capture the following frames
	for( int i =0 ;i!= frame_num; ++i )
	{
		if( points_file )
			points_file->navigateToNextFrame();
		if( flowers_viewer_ )
		{
			flowers_viewer_->next();
			flowers_viewer_->update();
		}
		Sleep(1000);

		sc->setCapture(true);
		sc->setUseFrameNumber(true);
		sc->setNumFramesToCapture(1);
		Sleep(500);
	}
	camera->setPostDrawCallback( old );	
	return true;

}
