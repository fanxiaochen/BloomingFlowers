#include <QDir>
#include <QProcess>
#include <QFileInfo>
#include <QMessageBox>
#include <QFileDialog>
#include <QMutexLocker>
#include <QProgressBar>
#include <QFutureWatcher>
#include <QtConcurrentFilter>

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include "mesh_model.h"
#include "main_window.h"
#include "point_cloud.h"
#include "scene_widget.h"
#include "parameter_manager.h"

#include "task_dispatcher.h"

namespace fs = boost::filesystem;

TaskImpl::TaskImpl(int id)
  :id_(id)
{}

TaskImpl::~TaskImpl(void)
{}

Task::Task(void)
{
}

Task::Task(TaskImpl* task_impl)
  :task_impl_(task_impl)
{
}

Task::Task(const Task &other)
{
  task_impl_ = other.task_impl_;
}

Task& Task::operator=(const Task &other)
{
  task_impl_ = other.task_impl_;
  return (*this);
}

Task::~Task()
{
}

bool Task::run(void) const
{
  task_impl_->run();

  emit finished(task_impl_->id_);

  return false;
}

TaskScaleModels::TaskScaleModels(int id, const std::string& filename, double expected_height)
	:TaskImpl(id),
	filename_(filename),
	expected_height_(expected_height)
{}

TaskScaleModels::~TaskScaleModels(void)
{}

void TaskScaleModels::run(void) const
{
	osg::ref_ptr<MeshModel> mesh_model = new MeshModel(id_);
	mesh_model->load(filename_);
	boost::filesystem::path path(filename_);
	mesh_model->scale(expected_height_);
	mesh_model->save(path.parent_path().string()+"/"+Common::int2String(id_, 4)+".ply");

	return;
}


TaskDispatcher::TaskDispatcher(QObject* parent)
  :QObject(parent)
{
}

TaskDispatcher::~TaskDispatcher(void)
{
  cancelRunningTasks(true);

  return;
}

bool TaskDispatcher::isRunning(void) const
{
  QMutexLocker locker(&mutex_);

  return (!active_watchers_.empty());
}

void TaskDispatcher::runTasksInParallel(QList<Task>& tasks, const std::string& task_name, bool display)
{
  QProgressBar* progress_bar = new QProgressBar(MainWindow::getInstance());
  progress_bar->setRange(0, tasks.size());
  progress_bar->setValue(0);
  progress_bar->setFormat(QString("%1: %p% completed").arg(task_name.c_str()));
  progress_bar->setTextVisible(true);
  MainWindow::getInstance()->statusBar()->addPermanentWidget(progress_bar);

  QFutureWatcher<void>* watcher = new QFutureWatcher<void>(this);
  active_watchers_.push_back(watcher);
  
  connect(watcher, SIGNAL(progressValueChanged(int)), progress_bar, SLOT(setValue(int)));
  connect(watcher, SIGNAL(finished()), progress_bar, SLOT(deleteLater()));
  connect(watcher, SIGNAL(finished()), this, SLOT(removeFinishedWatchers()));

  watcher->setFuture(QtConcurrent::filter(tasks, &Task::run));

  return;
}

void TaskDispatcher::runTasksSingleThreadImpl(QList<Task>& tasks)
{
  int progress = 0;
  for (QList<Task>::iterator it = tasks.begin(); it != tasks.end(); ++ it)
  {
    it->run();
    emit progressValueChanged(++progress);
  }
  tasks.clear();
  emit finished();

  return;
}

void TaskDispatcher::runTasksSingleThread(QList<Task>& tasks, const std::string& task_name, bool display)
{
  QProgressBar* progress_bar = new QProgressBar(MainWindow::getInstance());
  progress_bar->setRange(0, tasks.size());
  progress_bar->setValue(0);
  progress_bar->setFormat(QString("%1: %p% completed").arg(task_name.c_str()));
  progress_bar->setTextVisible(true);
  MainWindow::getInstance()->statusBar()->addPermanentWidget(progress_bar);

  connect(this, SIGNAL(progressValueChanged(int)), progress_bar, SLOT(setValue(int)));
  connect(this, SIGNAL(finished()), progress_bar, SLOT(deleteLater()));
  connect(this, SIGNAL(finished()), this, SLOT(removeFinishedWatchers()));

  boost::thread thread(&TaskDispatcher::runTasksSingleThreadImpl, this, boost::ref(tasks));
  thread.detach();

  return;
}

void TaskDispatcher::dispatchScaleModelsTasks(void)
{
	MainWindow* main_window = MainWindow::getInstance();
	if (!scale_models_tasks_.isEmpty())
	{
		QMessageBox::warning(main_window, "Scale Models Warning",
			"Run scale models task after the previous one has finished");
		return;
	}

	QString directory = QFileDialog::getExistingDirectory(main_window, "Scale Models in Directory", main_window->getWorkspace().c_str(), QFileDialog::ShowDirsOnly);
	if (directory.isEmpty())
		return;

	QDir dir(directory);
	QStringList obj_list = dir.entryList(QStringList("*.obj"), QDir::Files);
	if (obj_list.empty())
		return;

	double expected_height;
	if (!ParameterManager::getInstance().getExpectedHeightParameters(expected_height))
		return;

	int mesh_model_id = 0;
	std::ofstream fout(directory.toStdString()+"/mapping.txt");
	for (QStringList::iterator it = obj_list.begin(); it != obj_list.end(); ++ it)
	{
		QString filename = directory+"/"+*it;
		fout << QFileInfo(filename).baseName().toStdString() << "\t" << Common::int2String(mesh_model_id, 4) << std::endl;
		scale_models_tasks_.push_back(Task(new TaskScaleModels(mesh_model_id++, filename.toStdString(), expected_height)));
	}
	fout.close();

	// workaround boost filesystem path bug.
	osg::ref_ptr<MeshModel> mesh_model(new MeshModel(0));
	mesh_model->load((directory+"/"+obj_list.first()).toStdString());

	runTasksInParallel(scale_models_tasks_, "Scale Models");

	return;
}

void TaskDispatcher::cancelRunningTasks(bool wait)
{
  QMutexLocker locker(&mutex_);

  for (size_t i = 0, i_end = active_watchers_.size(); i < i_end; ++ i)
  {
    QFutureWatcher<void>* watcher = dynamic_cast<QFutureWatcher<void>*>(active_watchers_[i]);
    watcher->cancel();
    if (wait)
      watcher->waitForFinished();
  }

  return;
}

void TaskDispatcher::removeFinishedWatchers(void)
{
  QMutexLocker locker(&mutex_);

  std::vector<QObject*> temp;
  for (size_t i = 0, i_end = active_watchers_.size(); i < i_end; ++ i)
  {
    QFutureWatcher<void>* watcher = dynamic_cast<QFutureWatcher<void>*>(active_watchers_[i]);
    if (!watcher->isFinished())
      temp.push_back(active_watchers_[i]);
    else
      watcher->deleteLater();
  }

  active_watchers_ = temp;

  return;
}
