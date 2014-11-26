
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
#include "tracking_system.h"
#include "color_map.h"
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

        if (cluster_points_.empty())
		    colors->push_back(osg::Vec4(point.r / 255.0, point.g / 255.0, point.b / 255.0, 0));
        else
        {
            ClusterPoint cp = cluster_points_.at(i);
            osg::Vec4 color = ColorMap::getInstance().getDiscreteColor(cp._label + 1);
            colors->push_back(color);
        }
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

PointCloud* PointCloud::getPrevFrame(void)
{
    PointsFileSystem* model = dynamic_cast<PointsFileSystem*>(MainWindow::getInstance()->getPointsSystem());
    return model->getPointCloud(getFrame()-1);
}

PointCloud* PointCloud::getNextFrame(void)
{
    PointsFileSystem* model = dynamic_cast<PointsFileSystem*>(MainWindow::getInstance()->getPointsSystem());
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
  PointsFileSystem* model = dynamic_cast<PointsFileSystem*>(MainWindow::getInstance()->getPointsSystem());

  return model->isShown(filename_);
}

void PointCloud::pickEvent(int pick_mode, osg::Vec3 position)
{
    switch (pick_mode)
    {
    case (osgGA::GUIEventAdapter::MODKEY_CTRL):
        {
            const float eps = 1e-3;

            for (auto& it = picked_points_.begin(); it != picked_points_.end(); ++ it)
            {
                if (fabs(it->x() - position.x()) < eps &&
                    fabs(it->y() - position.y()) < eps &&
                    fabs(it->z() - position.z()) < eps)
                    return;
            }


            std::cout << "pick a point: " << std::endl;
            std::cout << position.x() << " " << position.y() << " " << position.z() << std::endl << std::endl;


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


void PointCloud::petal_segmentation()
{
    k_means();
    return;
}

void PointCloud::k_means()
{
    int cluster_num = picked_indices_.size();

    computeClusterPoints();

    setCenters();

    std::vector<ClusterPoint> next_centers;

    do 
    {
        if (!next_centers.empty())
        {
            cluster_centers_ = next_centers;
            next_centers.clear();
        }

        size_t points_num = cluster_points_.size();
        for (size_t i = 0; i < points_num; i ++)
        {
            int cluster_id = determineCluster(cluster_points_[i]);
            cluster_points_[i]._label = cluster_id;
        }

        for (size_t i = 0; i < cluster_num; i ++)
        {
            std::vector<int> ids;
            for (size_t j = 0; j < points_num; j ++)
            {
                if (cluster_points_[j]._label == i)
                    ids.push_back(j);
            }

            ClusterPoint next_center = mean_center(ids);
            next_centers.push_back(next_center);
        }

        updateCenters();
        expire();

    } while (!terminal(cluster_centers_, next_centers));
}

void PointCloud::computeClusterPoints()
{
    PointsFileSystem* points_file_system = dynamic_cast<PointsFileSystem*>(MainWindow::getInstance()->getPointsSystem());
    TrackingSystem* tracking_system = new TrackingSystem(points_file_system);
    PointCloud* next_cloud = getNextFrame();

    std::vector<int> src_idx, tar_idx;
    for (size_t i = 0, i_end = this->size(); i < i_end; ++ i)
        src_idx.push_back(i);

    tracking_system->cpd_registration(*this, *next_cloud, src_idx, tar_idx);
    
    for (size_t i = 0, i_end = this->size(); i < i_end; ++ i)
    {
        ClusterPoint cp;

        osg::Vec3 mv(next_cloud->at(i).x - this->at(i).x, 
            next_cloud->at(i).y - this->at(i).y,
            next_cloud->at(i).z - this->at(i).z);
        mv.normalize();
        cp._mv.x() = mv.x();
        cp._mv.y() = mv.y();
        cp._mv.z() = mv.z();

        cp._label = -1;
        cp._pt = this->at(i);

        cluster_points_.push_back(cp);
    }
}

void PointCloud::setCenters()
{
    for (size_t i = 0, i_end = picked_indices_.size(); i < i_end; ++ i)
    {
        cluster_centers_.push_back(cluster_points_[picked_indices_[i]]);
    }
}

void PointCloud::updateCenters()
{
    for (size_t i = 0, i_end = picked_points_.size(); i < i_end; ++i)
    {
        ClusterPoint cp = cluster_centers_.at(i);
        picked_points_[i].x() = cp._pt.x;
        picked_points_[i].y() = cp._pt.y;
        picked_points_[i].z() = cp._pt.z;
    }
}

float PointCloud::distance(const ClusterPoint& p1, const ClusterPoint& p2)
{
    float lambda = 1.0;

    float euc_dist = pow(p1._pt.x - p2._pt.x, 2.0) + pow(p1._pt.y - p2._pt.y, 2.0) +
        pow(p1._pt.z - p2._pt.z, 2.0);
    float mv_dist = (p1._mv - p2._mv).length2();

    float dist = euc_dist * (1 + lambda * mv_dist);

    return dist;
}

int PointCloud::determineCluster(const ClusterPoint& point)
{
    int cluster_num = cluster_centers_.size();

    float min = std::numeric_limits<float>::max();
    int cluster_id = -1;

    for (size_t i = 0, i_end = cluster_num; i < i_end; i ++)
    {
        float temp_dist = distance(point, cluster_centers_[i]);
        if (min > temp_dist)
        {
            min = temp_dist;
            cluster_id = i;
        }
    }

    return cluster_id;
}

PointCloud::ClusterPoint PointCloud::mean_center(const std::vector<int>& ids)
{
    ClusterPoint cp;

    for (size_t i = 0, i_end = ids.size(); i < i_end; i ++)
    {
        ClusterPoint tmp_cp = cluster_points_.at(ids[i]);
        
        cp = cp + tmp_cp;
    }

    cp = cp / ids.size();

    return cp;
}

bool PointCloud::terminal(const std::vector<ClusterPoint>& cluster_centers, const std::vector<ClusterPoint>& next_centers)
{
    int cluster_num = cluster_centers_.size();

    const float eps = 1e-3;

    for (size_t i = 0; i < cluster_num; i ++)
    {
        float delta = distance(cluster_centers[i], next_centers[i]);
        if (delta > eps)
            return false;
    }

    return true;
}


