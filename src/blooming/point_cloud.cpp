
#include <QRegExp>
#include <QFileInfo>
#include <QDockWidget>
#include <QFileDialog>

#include <pcl/io/pcd_io.h>

#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/Point>

#include "main_window.h"
#include "points_file_system.h"
#include "osg_viewer_widget.h"
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


    for (auto& it = picked_points_.begin(); it != picked_points_.end(); it ++)
    {
        osg::Geode* picked_geode = new osg::Geode();
        osg::Sphere* sphere = new osg::Sphere(*it, 5.0);
        osg::ShapeDrawable* drawable = new osg::ShapeDrawable(sphere);
        drawable->setColor(osg::Vec4(1.0,0.0,0.0,0.0));
        picked_geode->addDrawable(drawable);
        content_root_->addChild(picked_geode);
    }

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

void PointCloud::pickEvent(int pick_mode, osg::Vec3 position)
{
    switch (pick_mode)
    {
    case (osgGA::GUIEventAdapter::MODKEY_CTRL):
        {
            std::cout << "pick a point: " << std::endl;
            std::cout << position.x() << " " << position.y() << " " << position.z() << std::endl << std::endl;

            const float eps = 1e-3;
            for (auto& it = this->begin(); it != this->end(); it ++)
            {
                if (fabs(it->x - position.x()) < eps &&
                    fabs(it->y - position.y()) < eps &&
                    fabs(it->z - position.z()) < eps)
                    picked_indices_.push_back(it - this->begin());
            }

            picked_points_.push_back(position);

            expire();

        }
    default:
        break;
    }
}
