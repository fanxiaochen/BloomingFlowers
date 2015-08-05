
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
    :segmented_(false),
    show_boundary_(false),
    show_tips_(false)
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
    // for points
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
            colors->push_back(ColorMap::getInstance().getDiscreteColor(segment_flags_[i]));
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
   

    // for tips
    if (show_tips_)
    {
        for (size_t i = 0, i_end = tip_indices_.size(); i < i_end; ++ i)
        {
            // create a sphere for each point
            const Point& p = at(tip_indices_[i]);
            osg::Vec3 point(p.x, p.y, p.z);
            osg::Sphere* sphere = new osg::Sphere(point, 2);
            osg::ShapeDrawable* drawable = new osg::ShapeDrawable(sphere);
            drawable->setColor(ColorMap::getInstance().getDiscreteColor(16));
            geode->addDrawable(drawable);
        }
    }

    // for boundary
    if (show_boundary_)
    {
        osg::ref_ptr<osg::Vec3Array>  bvertices = new osg::Vec3Array;
        osg::ref_ptr<osg::Vec4Array>  bcolors = new osg::Vec4Array;
        for (size_t i = 0, i_end = boundary_indices_.size(); i < i_end; ++ i)
        {
            const Point& point = at(boundary_indices_[i]);
            bvertices->push_back(osg::Vec3(point.x, point.y, point.z));
            bcolors->push_back(ColorMap::getInstance().getDiscreteColor(8));

            //// create a sphere for each point
            //const Point& p = at(boundary_indices_[i]);
            //osg::Vec3 point(p.x, p.y, p.z);
            //osg::Sphere* sphere = new osg::Sphere(point, 1);
            //osg::ShapeDrawable* drawable = new osg::ShapeDrawable(sphere);
            //drawable->setColor(ColorMap::getInstance().getDiscreteColor(8));
            //geode->addDrawable(drawable);
        }
        osg::Geometry* bgeometry = new osg::Geometry;
        bgeometry->setVertexArray(bvertices);
        bgeometry->setColorArray(bcolors);
        bgeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
        bgeometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, bvertices->size()));
        bgeometry->getOrCreateStateSet()->setAttribute(new osg::Point(10.0f));
        geode->addDrawable(bgeometry);
    }
    
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

void PointCloud::region_growing(std::vector<int>& segment_index, int petal_id)
{
    std::unordered_set<int> segment_set(segment_index.begin(), segment_index.end());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0, i_end = this->size(); i < i_end; ++ i)
    {
        const Point& point = this->at(i);

        pcl::PointXYZ pcl_point(point.x, point.y, point.z);
        cloud->push_back(pcl_point);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud (cloud);

    float growing_radius = 2;

    // radius neighbor search 

    for (int k = 0; k < segment_index.size(); ++ k)
    {
        pcl::PointXYZ searchPoint;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        searchPoint.x = this->at(segment_index[k]).x;
        searchPoint.y = this->at(segment_index[k]).y;
        searchPoint.z = this->at(segment_index[k]).z;

        if ( kdtree.radiusSearch (searchPoint, growing_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
            bool growing = true;
            for (int i = 0, i_end = pointIdxRadiusSearch.size(); i < i_end; ++ i)
            {
                if (segment_flags_[pointIdxRadiusSearch[i]] != -1 && segment_flags_[pointIdxRadiusSearch[i]] != petal_id)
                {
                    growing = false;
                    break;
                }
            }

            if (growing)
            {
                for (int i = 0, i_end = pointIdxRadiusSearch.size(); i < i_end; ++ i)
                {
                    if (segment_set.find(pointIdxRadiusSearch[i]) == segment_set.end())
                    {
                        segment_flags_[pointIdxRadiusSearch[i]] = petal_id;
                        segment_index.push_back(pointIdxRadiusSearch[i]);
                        segment_set.insert(pointIdxRadiusSearch[i]);
                    }
                }
            }
        }
    }
}