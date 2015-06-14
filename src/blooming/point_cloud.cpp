
#include <QRegExp>
#include <QFileInfo>
#include <QDockWidget>
#include <QFileDialog>

#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/Point>

#include <pcl/kdtree/kdtree_flann.h>

#include "main_window.h"
#include "points_file_system.h"
#include "osg_viewer_widget.h"
#include "tracking_system.h"
#include "color_map.h"
#include "point_cloud.h"
#include "flower.h"

PointCloud::PointCloud(void)
    :segmented_(false)
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
    Renderable::clear();
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

    for (size_t i = 0, i_end = size(); i < i_end; i ++)
    {
        if (!segmented_)
        {
            const Point& point = at(i);
            vertices->push_back(osg::Vec3(point.x, point.y, point.z));
            colors->push_back(osg::Vec4(point.r / 255.0, point.g / 255.0, point.b / 255.0, 0));
        }
        else 
        {
            const Point& point = at(i);
            vertices->push_back(osg::Vec3(point.x, point.y, point.z));
            osg::Vec4 color = ColorMap::getInstance().getDiscreteColor(segment_flags_[i]);
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

void PointCloud::searchNearestIdx(MeshModel* mesh_model, std::vector<int>& knn_idx, std::vector<float>& knn_dists)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0, i_end = mesh_model->getVertices()->size(); i < i_end; ++ i)
    {
        // we only use visible vertices to segment point cloud
        if (mesh_model->getVisibility().size() != 0 && mesh_model->getVisibility()[i] == 0)
        {
             pcl::PointXYZ pcl_point(0.0f, 0.0f, 0.0f);  // origin point is far from our dataset
             cloud->push_back(pcl_point);
             continue;
        }

        const osg::Vec3& point = mesh_model->getVertices()->at(i);
        pcl::PointXYZ pcl_point(point.x(), point.y(), point.z());
        cloud->push_back(pcl_point);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud (cloud);

    int K = 1;

    // K nearest neighbor search

    for (size_t i = 0, i_end = this->size(); i < i_end; ++ i)
    {
        pcl::PointXYZ searchPoint;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        Point& point = this->at(i);

        searchPoint.x = point.x;
        searchPoint.y = point.y;
        searchPoint.z = point.z;

        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            knn_idx.push_back(pointIdxNKNSearch[0]);
            knn_dists.push_back(pointNKNSquaredDistance[0]);
        }
    }
}

osg::ref_ptr<PointCloud> PointCloud::getPetalCloud(int id)
{
    osg::ref_ptr<PointCloud> petal_cloud = new PointCloud;

    for (size_t i = 0, i_end = segment_flags_.size(); i < i_end; ++ i)
    {
        if (segment_flags_[i] == id)
            petal_cloud->push_back(this->at(i));
    }

    if (petal_cloud->size() == 0)
        return NULL;

    return petal_cloud;
}

void PointCloud::flower_segmentation(Flower* flower)
{
    segment_flags_.clear();

    std::vector<std::vector<int> > knns_idx;
    std::vector<std::vector<float> > knns_dists;

    for (size_t i = 0, i_end = flower->getPetals().size(); i < i_end; ++ i)
    {
        std::vector<int> knn_idx;
        std::vector<float> knn_dists;

        Petal& petal = flower->getPetals().at(i);

        searchNearestIdx(&petal, knn_idx, knn_dists);

        knns_idx.push_back(knn_idx);
        knns_dists.push_back(knn_dists);
    }


    for (size_t i = 0, i_end = this->size(); i < i_end; ++ i)
    {
        float min_dist = std::numeric_limits<float>::max();
        int min_j = std::numeric_limits<int>::max();

        for (size_t j = 0, j_end = knns_idx.size(); j < j_end; ++ j)
        {
            if (min_dist > knns_dists[j][i])
            {
                min_j = j;
                min_dist = knns_dists[j][i];
            }
        }

        segment_flags_.push_back(min_j);
    }

    segmented_ = true;
}