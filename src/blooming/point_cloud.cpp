
#include "lily.h"

#include <QRegExp>
#include <QFileInfo>
#include <QDockWidget>
#include <QFileDialog>

#include <pcl/io/pcd_io.h>

#include <osg/Geometry>
#include <osg/Point>

#include "point_cloud.h"

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
	PointCloud::clear();

	return;
}


void PointCloud::updateImpl()
{
	visualizePoints();
	return;
}

void PointCloud::visualizePoints()
{
	osg::ref_ptr<osg::Vec3Array>  vertices = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array>  colors = new osg::Vec4Array;

	for (size_t i= 0, i_end = size(); i < i_end; i ++)
	{
		const Point& point = at(i);

		vertices->push_back(osg::Vec3(point.x, point.y, point.z));
		colors->push_back(osg::Vec4(point.r / 255.0, point.g / 255.0, point.b / 255.0, 0));
	}

	osg::Geometry* geometry = new osg::Geometry;
	geometry->setVertexArray(vertices);
	geometry->setColorArray(colors);
	geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, size()));
	geometry->getOrCreateStateSet()->setAttribute(new osg::Point(5.0f));
	osg::Geode* geode = new osg::Geode;
	geode->addDrawable(geometry);
	content_root_->addChild(geode);

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
  PointsFileSystem* model = dynamic_cast<PointsFileSystem*>(MainWindow::getInstance()->getPointsSystem());

  return model->isShown(filename_);
}
