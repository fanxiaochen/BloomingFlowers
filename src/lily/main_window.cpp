
#include <QToolTip>
#include <QKeyEvent>
#include <QSettings>
#include <QGridLayout>
#include <QDockWidget>
#include <QFileDialog>
#include <QMessageBox>
#include <QApplication>

#include "main_window.h"
#include "file_viewer_widget.h"
#include "scene_widget.h"
#include "points_file_system.h"
#include "mesh_file_system.h"
#include "tracking_system.h"
#include "parameters.h"
#include "trajectory_model.h"


MainWindow::MainWindow(void)
    :points_path_("."),
    mesh_path_("."),
    points_files_(NULL),
    mesh_files_(NULL),
    points_widget_(NULL),
    mesh_widget_(NULL),
    tracking_system_(NULL),
    parameters_(NULL)
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
            points_file->navigateToPreviousFrame();
            break;
    case(Qt::Key_Down):
            points_file->navigateToNextFrame();
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
    

    scene_widget_ = new SceneWidget(this);
    setCentralWidget(scene_widget_);
    scene_widget_->startRendering();

    connect(this, SIGNAL(showInformationRequested(const QString&)), this, SLOT(slotShowInformation(const QString&)));
    connect(this, SIGNAL(showStatusRequested(const QString&, int)), this, SLOT(slotShowStatus(const QString&, int)));
    loadSettings();

    connect(ui_.actionLoadPoints, SIGNAL(triggered()), this, SLOT(slotLoadPoints()));
    connect(ui_.actionLoadMesh, SIGNAL(triggered()), this, SLOT(slotLoadMesh()));
    connect(ui_.actionLoadTrajectories, SIGNAL(triggered()), this, SLOT(slotLoadTrajectories()));
    connect(ui_.actionLoadParameters, SIGNAL(triggered()), this, SLOT(slotLoadParameters()));
    connect(ui_.actionSaveParameters, SIGNAL(triggered()), this, SLOT(slotSaveParameters()));


    connect(ui_.actionPointCloudTracking, SIGNAL(triggered()), tracking_system_, SLOT(pointcloud_tracking()));
    connect(ui_.actionMeshTracking, SIGNAL(triggered()), tracking_system_, SLOT(mesh_tracking()));
    connect(ui_.actionTrajectoryTracking, SIGNAL(triggered()), tracking_system_, SLOT(buildTrajectories()));
    connect(ui_.actionFlowerTracking, SIGNAL(triggered()), tracking_system_, SLOT(flower_tracking()));

    connect(ui_.actionKMeansForTrajectories, SIGNAL(triggered()), tracking_system_, SLOT(clusterTrajectories()));
    connect(ui_.actionKMeansForFlower, SIGNAL(triggered()), points_files_, SLOT(segmentation()));
    connect(ui_.actionPropagateSegments, SIGNAL(triggered()), tracking_system_, SLOT(propagateSegments()));

    // connect

    return;
}

bool MainWindow::slotLoadPoints(void)
{
    QString directory = QFileDialog::getExistingDirectory(this, tr("Load Points"), points_path_.c_str(), QFileDialog::ShowDirsOnly);

    if (directory.isEmpty())
        return false;

    points_path_ = directory.toStdString();

    points_widget_->setWorkspace(directory);

    directory.resize(directory.size()-7); // remove string "/points", the workspace is based on points path
    workspace_ = directory.toStdString();

    return true;
}

bool MainWindow::slotLoadMesh(void)
{
    QString directory = QFileDialog::getExistingDirectory(this, tr("Load Mesh"), mesh_path_.c_str(), QFileDialog::ShowDirsOnly);

    if (directory.isEmpty())
        return false;

    mesh_path_ = directory.toStdString();

    mesh_widget_->setWorkspace(directory);

    return true;
}

bool MainWindow::slotLoadTrajectories()
{
    QString traj_file = QFileDialog::getOpenFileName(this, tr("Load Trajectories"), workspace_.c_str(),  tr("Files (*.json)"));

    if (traj_file.isEmpty())
        return false;

    tracking_system_->getTrajectories()->load(traj_file.toStdString());
    MainWindow::getInstance()->getSceneWidget()->addSceneChild(tracking_system_->getTrajectories());

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

void MainWindow::loadSettings()
{
    QSettings settings("Blooming Flower", "Blooming Flower");

    points_path_ = settings.value("workspace").toString().toStdString();

    return;
}

void MainWindow::saveSettings()
{
    QSettings settings("Blooming Flower", "Blooming Flower");

    QString workspace(points_path_.c_str());
    settings.setValue("workspace", workspace);

    return;
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