
#include <QRegExp>
#include <QFileInfo>
#include <QDockWidget>
#include <QFileDialog>

#include <pcl/io/pcd_io.h>

#include "main_window.h"
#include "file_system_model.h"
#include "point_cloud.h"
#include "osg_viewer_widget.h"

PointCloud::PointCloud(void)
{
  
}

PointCloud::~PointCloud(void)
{
}

bool PointCloud::open(const std::string& filename)
{
  clearData();

  if (pcl::io::loadPCDFile(filename, *this) != 0)
    return false;

  filename_ = filename;

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

  //Renderable::clear();
  PclPointCloud::clear();

  return;
}


void PointCloud::updateImpl()
{
  // visualize point cloud
  return;
}

//PointCloud* PointCloud::getPrevFrame(void)
//{
//  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
//  return model->getPointCloud(getFrame()-1);
//}
//
//PointCloud* PointCloud::getNextFrame(void)
//{
//  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
//  return model->getPointCloud(getFrame()+1);
//}


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
