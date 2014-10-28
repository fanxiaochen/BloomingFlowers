#include <QToolTip>
#include <QKeyEvent>
#include <QSettings>
#include <QGridLayout>
#include <QDockWidget>
#include <QFileDialog>
#include <QMessageBox>
#include <QApplication>

#include "scene_widget.h"
#include "main_window.h"

MainWindow::MainWindow(void)
  :workspace_(".")
{
	ui_.setupUi(this);

	MainWindowInstancer::getInstance().main_window_ = this;

	init();
}

MainWindow::~MainWindow()
{
	saveSettings();

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
	if (event->key() == Qt::Key_Down)
	{
		emit keyDownPressed();
	}

	QMainWindow::keyPressEvent(event);

	return;
}

MainWindow* MainWindow::getInstance()
{
	assert(MainWindowInstancer::getInstance().main_window_ != NULL);
	return MainWindowInstancer::getInstance().main_window_;
}

SceneWidget* MainWindow::getSceneWidget(void)
{
	return dynamic_cast<SceneWidget*>(centralWidget());
}

void MainWindow::init(void)
{

	SceneWidget* scene_widget = new SceneWidget(this);
	setCentralWidget(scene_widget);
	scene_widget->startRendering();

	connect(this, SIGNAL(showInformationRequested(const QString&)), this, SLOT(slotShowInformation(const QString&)));
	connect(this, SIGNAL(showStatusRequested(const QString&, int)), this, SLOT(slotShowStatus(const QString&, int)));

	loadSettings();

	//// File
	//connect(ui_.actionOpenPointCloud, SIGNAL(triggered()), scene_widget, SLOT(slotOpenPointCloud()));
	//connect(ui_.actionSavePointCloud, SIGNAL(triggered()), scene_widget, SLOT(slotSavePointCloud()));
	//connect(ui_.actionSetWorkspace, SIGNAL(triggered()), this, SLOT(slotSetWorkspace()));
	//connect(ui_.actionPOVRaySnapshot, SIGNAL(triggered()), scene_widget, SLOT(slotPOVRaySnapshot()));
	//connect(ui_.actionPOVRayVideo, SIGNAL(triggered()), scene_widget, SLOT(slotPOVRayVideo()));
	//connect(ui_.actionImportMeshModels, SIGNAL(triggered()), scene_widget, SLOT(slotImportMeshModels()));
	//connect(ui_.actionSampleMeshModels, SIGNAL(triggered()), scene_widget, SLOT(slotSampleMeshModels()));
	//connect(ui_.actionVirtualScan, SIGNAL(triggered()), scene_widget, SLOT(slotVirtualScan()));
	//connect(ui_.actionClearMeshModels, SIGNAL(triggered()), scene_widget, SLOT(slotClearMeshModels()));

	//// Algorithms
	//connect(ui_.actionNormalEstimation, SIGNAL(triggered()), scene_widget, SLOT(slotEstimateNormals()));
	//connect(ui_.actionCurvatureEstimation, SIGNAL(triggered()), scene_widget, SLOT(slotEstimateCurvatures()));
	//connect(ui_.actionNormalOrientation, SIGNAL(triggered()), scene_widget, SLOT(slotOrientNormals()));
	//connect(ui_.actionFlipAllNormals, SIGNAL(triggered()), scene_widget, SLOT(slotFlipAllNormals()));
	//connect(ui_.actionVoxelGridFilter, SIGNAL(triggered()), scene_widget, SLOT(slotVoxelGridFilter()));

	//// Visualization
	//connect(ui_.actionTogglePointCloud, SIGNAL(triggered()), scene_widget, SLOT(slotTogglePointCloud()));
	//connect(ui_.actionToggleCloudNormals, SIGNAL(triggered()), scene_widget, SLOT(slotToggleShowNormals()));
	//connect(ui_.actionToggleCloudDraggers, SIGNAL(triggered()), scene_widget, SLOT(slotToggleCloudDraggers()));
	//connect(ui_.actionToggleCloudScalers, SIGNAL(triggered()), scene_widget, SLOT(slotToggleCloudScalers()));
	//connect(ui_.actionToggleMeshModels, SIGNAL(triggered()), scene_widget, SLOT(slotToggleMeshModels()));
	//connect(ui_.actionColorizeOriginal, SIGNAL(triggered()), scene_widget, SLOT(slotColorizeOriginal()));
	//connect(ui_.actionColorizeUniform, SIGNAL(triggered()), scene_widget, SLOT(slotColorizeUniform()));
	//connect(ui_.actionColorizeCategory, SIGNAL(triggered()), scene_widget, SLOT(slotColorizeCategory()));
	//connect(ui_.actionColorizeInstance, SIGNAL(triggered()), scene_widget, SLOT(slotColorizeInstance()));
	//connect(ui_.actionColorizeSegment, SIGNAL(triggered()), scene_widget, SLOT(slotColorizeSegment()));
	//connect(ui_.actionColorizeCurvature, SIGNAL(triggered()), scene_widget, SLOT(slotColorizeCurvature()));
	//connect(ui_.actionColorizeDepth, SIGNAL(triggered()), scene_widget, SLOT(slotColorizeDepth()));
	//connect(ui_.actionColorizeIntensity, SIGNAL(triggered()), scene_widget, SLOT(slotColorizeIntensity()));
	//connect(ui_.actionSensorFromCamera, SIGNAL(triggered()), scene_widget, SLOT(slotSensorFromCamera()));
	//connect(ui_.actionCameraFromSensor, SIGNAL(triggered()), scene_widget, SLOT(slotCameraFromSensor()));

	return;
}

bool MainWindow::slotSetWorkspace(void)
{
	QString directory = QFileDialog::getExistingDirectory(this, tr("Set Workspace"), workspace_.c_str(), QFileDialog::ShowDirsOnly);

	if (directory.isEmpty())
		return false;

	workspace_ = directory.toStdString();

	return true;
}

void MainWindow::loadSettings()
{
	QSettings settings("Blooming Flower", "Blooming Flower");

	workspace_ = settings.value("workspace").toString().toStdString();

	return;
}

void MainWindow::saveSettings()
{
	QSettings settings("Blooming Flower", "Blooming Flower");

	QString workspace(workspace_.c_str());
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