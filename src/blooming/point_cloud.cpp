#include <fstream>

#include <QRegExp>
#include <QFileInfo>
#include <QDockWidget>
#include <QFileDialog>
#include <QColorDialog>
#include <QFutureWatcher>
#include <QtConcurrentRun>

#include <osg/Geode>
#include <osg/io_utils>
#include <osg/Geometry>
#include <osgManipulator/TrackballDragger>
#include <osgManipulator/TranslateAxisDragger>


#include "main_window.h"
#include "file_system_model.h"
#include "osg_viewer_widget.h"
#include "renderable.h"
#include "point_cloud.h"

PointCloud::PointCloud(void)
 /* :translate_dragger_(new osgManipulator::TranslateAxisDragger),
  trackball_dragger_(new osgManipulator::TrackballDragger)*/
{
  /*translate_dragger_->setupDefaultGeometry();
  translate_dragger_->setHandleEvents(true);
  translate_dragger_->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL);
  translate_dragger_->setActivationKeyEvent('d');

  trackball_dragger_->setupDefaultGeometry();
  trackball_dragger_->setHandleEvents(true);
  trackball_dragger_->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL);
  trackball_dragger_->setActivationKeyEvent('d');

  translate_dragger_->addTransformUpdating(this);
  translate_dragger_->addTransformUpdating(trackball_dragger_);

  trackball_dragger_->addTransformUpdating(this);
  trackball_dragger_->addTransformUpdating(translate_dragger_);*/
}

PointCloud::~PointCloud(void)
{
}

bool PointCloud::open(const std::string& filename)
{
  clearData();

  QMutexLocker locker(&mutex_);

  if (pcl::io::loadPCDFile(filename, *this) != 0)
    return false;

  filename_ = filename;

  locker.unlock();
  loadStatus();
  locker.relock();

  expire();

  return true;
}

void PointCloud::reload(void)
{
  clearData();
  open(filename_);

  return;
}

void PointCloud::clearData()
{
  QMutexLocker locker(&mutex_);

  Renderable::clear();
  PclRichPointCloud::clear();

  return;
}


void PointCloud::updateImpl()
{
  // visualize point cloud
  return;
}

PointCloud* PointCloud::getPrevFrame(void)
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  return model->getPointCloud(getFrame()-1);
}

PointCloud* PointCloud::getNextFrame(void)
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  return model->getPointCloud(getFrame()+1);
}


int PointCloud::getFrame(void) const      
{
  QRegExp frame("[\\/]frame_([0-9]{5,5})[\\/]");
  frame.indexIn(filename_.c_str());
  QString index = frame.cap(1);

  return index.toInt();
}


bool PointCloud::isShown(void) const
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();

  return model->isShown(filename_);
}
