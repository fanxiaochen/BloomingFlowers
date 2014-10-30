#include <QToolTip>
#include <QKeyEvent>
#include <QSettings>
#include <QGridLayout>
#include <QDockWidget>
#include <QFileDialog>
#include <QMessageBox>
#include <QApplication>

#include "main_window.h"

MainWindow::MainWindow(void)
	:workspace_("."),
	file_system_model_(NULL),
	file_viewer_widget_(NULL)
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

void MainWindow::init(void)
{
	file_system_model_ = new FileSystemModel;
	file_viewer_widget_ = new FileViewerWidget(this, file_system_model_);

	QDockWidget* dock_widget_file_viewer = new QDockWidget("File Viewer", this);
	addDockWidget(Qt::LeftDockWidgetArea, dock_widget_file_viewer);
	dock_widget_file_viewer->setAllowedAreas(Qt::LeftDockWidgetArea|Qt::RightDockWidgetArea);

	file_viewer_widget_->setParent(dock_widget_file_viewer);
	dock_widget_file_viewer->setWidget(file_viewer_widget_);
	

	SceneWidget* scene_widget = new SceneWidget(this);
	setCentralWidget(scene_widget);
	scene_widget->startRendering();

	connect(this, SIGNAL(showInformationRequested(const QString&)), this, SLOT(slotShowInformation(const QString&)));
	connect(this, SIGNAL(showStatusRequested(const QString&, int)), this, SLOT(slotShowStatus(const QString&, int)));
	loadSettings();

	connect(ui_.actionSetWorkspace, SIGNAL(triggered()), this, SLOT(slotSetWorkspace()));
	// connect

	return;
}

bool MainWindow::slotSetWorkspace(void)
{
	QString directory = QFileDialog::getExistingDirectory(this, tr("Set Workspace"), workspace_.c_str(), QFileDialog::ShowDirsOnly);

	if (directory.isEmpty())
		return false;

	workspace_ = directory.toStdString();

	file_viewer_widget_->setWorkspace(directory);

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