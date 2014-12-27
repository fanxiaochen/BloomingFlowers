
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
            /*const FlowerPoint& point = flower_points_.at(i);
            vertices->push_back(osg::Vec3(point._pt.x, point._pt.y, point._pt.z));
            osg::Vec4 color = ColorMap::getInstance().getDiscreteColor(point._label + 1);
            colors->push_back(color);*/

            const Point& point = at(i);
            vertices->push_back(osg::Vec3(point.x, point.y, point.z));
            osg::Vec4 color = ColorMap::getInstance().getDiscreteColor(segment_flags_[i] + 1);
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

void PointCloud::resetSegmentation()
{
    picked_indices_.clear();
    picked_points_.clear();
    cluster_centers_.clear();

    for (auto& it = flower_points_.begin(); it != flower_points_.end(); ++ it)
    {
        it->_label = -1;
    }

    segmented_ = false;
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

    if (cluster_num == 0)
        return;

    segmented_ = true;

    computeFlowerPoints();

    setCenters();

    std::vector<FlowerPoint> next_centers;

    do 
    {
        if (!next_centers.empty())
        {
            cluster_centers_ = next_centers;
            next_centers.clear();
        }

        size_t points_num = flower_points_.size();
        for (size_t i = 0; i < points_num; i ++)
        {
            int cluster_id = determineCluster(flower_points_[i]);
            flower_points_[i]._label = cluster_id;
        }

        for (size_t i = 0; i < cluster_num; i ++)
        {
            std::vector<int> ids;
            for (size_t j = 0; j < points_num; j ++)
            {
                if (flower_points_[j]._label == i)
                    ids.push_back(j);
            }

            FlowerPoint next_center = mean_center(ids);
            next_centers.push_back(next_center);
        }

        updateCenters();
        expire();

    } while (!terminal(cluster_centers_, next_centers));
}

void PointCloud::computeFlowerPoints()
{
    for (size_t i = 0, i_end = this->size(); i < i_end; ++ i)
    {
        FlowerPoint fp;

        fp._label = -1;
        fp._pt = this->at(i);

        flower_points_.push_back(fp);
    }
}

void PointCloud::setCenters()
{
    for (size_t i = 0, i_end = picked_indices_.size(); i < i_end; ++ i)
    {
        cluster_centers_.push_back(flower_points_[picked_indices_[i]]);
    }
}

void PointCloud::updateCenters()
{
    for (size_t i = 0, i_end = picked_points_.size(); i < i_end; ++i)
    {
        FlowerPoint cp = cluster_centers_.at(i);
        picked_points_[i].x() = cp._pt.x;
        picked_points_[i].y() = cp._pt.y;
        picked_points_[i].z() = cp._pt.z;
    }
}

float PointCloud::distance(const FlowerPoint& p1, const FlowerPoint& p2)
{

    float euc_dist = pow(p1._pt.x - p2._pt.x, 2.0) + pow(p1._pt.y - p2._pt.y, 2.0) +
        pow(p1._pt.z - p2._pt.z, 2.0);

    return euc_dist;
}

int PointCloud::determineCluster(const FlowerPoint& point)
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

PointCloud::FlowerPoint PointCloud::mean_center(const std::vector<int>& ids)
{
    FlowerPoint cp;

    for (size_t i = 0, i_end = ids.size(); i < i_end; i ++)
    {
        FlowerPoint tmp_cp = flower_points_.at(ids[i]);

        cp = cp + tmp_cp;
    }

    cp = cp / ids.size();

    return cp;
}

bool PointCloud::terminal(const std::vector<FlowerPoint>& cluster_centers, const std::vector<FlowerPoint>& next_centers)
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


void PointCloud::reordering(osg::ref_ptr<PointCloud> point_cloud, const std::vector<int>& idx)
{    
    for (size_t i = 0, i_end = idx.size(); i < i_end; ++ i)
    {
        point_cloud->push_back(this->at(idx[i]));
    }
}

void PointCloud::searchNearestIdx(MeshModel* mesh_model, std::vector<int>& knn_idx, std::vector<float>& knn_dists)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0, i_end = mesh_model->getVertices()->size(); i < i_end; ++ i)
    {
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

void PointCloud::segmentation_by_flower(Flower* flower)
{
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