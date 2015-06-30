
#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <QObject>
#include <QColor>
#include <osg/Array>
#include <unordered_set>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "renderable.h"
   
typedef  pcl::PointXYZRGB        Point;
typedef  pcl::PointCloud<Point>  PclPointCloud;

// pcl::PointXYZRGB could not be inherited and used as template of PointCloud class in pcl, I have to write 
// operator overloads outside...

static Point operator + (const Point& p1, const Point& p2)
{
    Point p;
    p.x = p1.x + p2.x;
    p.y = p1.y + p2.y;
    p.z = p1.z + p2.z;

    return p;
}

static Point operator - (const Point& p1, const Point& p2)
{
    Point p;
    p.x = p1.x - p2.x;
    p.y = p1.y - p2.y;
    p.z = p1.z - p2.z;

    return p;
}

static Point operator * (const Point& p, float mul)
{
    Point point;
    point.x = p.x * mul;
    point.y = p.y * mul;
    point.z = p.z * mul;

    return point;
}

static Point operator / (const Point& p, float div)
{
    Point point;
    point.x = p.x / div;
    point.y = p.y / div;
    point.z = p.z / div;

    return point;
}

static bool operator == (const Point& p1, const Point& p2)
{
    const float eps = 1e-3;

    if (fabs(p1.x - p2.x) < eps&&
        fabs(p1.y - p2.y) < eps&&
        fabs(p1.z - p2.z) < eps)
        return true;
    else
        return false;
}

static bool operator != (const Point& p1, const Point& p2)
{
    const float eps = 1e-3;

    if (fabs(p1.x - p2.x) > eps ||
        fabs(p1.y - p2.y) > eps ||
        fabs(p1.z - p2.z) > eps)
        return true;
    else
        return false;
}


class Flower;
class MeshModel;

class PointCloud : public QObject, public Renderable, public PclPointCloud
{

public:
  PointCloud(void);
  virtual ~PointCloud(void);

 // virtual const char* className() const {return "PointCloud";}

  bool open(const std::string& filename);
  void reload(void);

  inline const std::string& getFilename(void) const {return filename_;}
  
  int getFrame(void) const;
  bool isShown(void) const;
  
  // template segmentation for em tracking framework
  void flower_segmentation(Flower* flower);

  osg::ref_ptr<PointCloud> getPetalCloud(int id);

  std::vector<int>& getSegmentFlags() { return segment_flags_; }
protected:
  virtual void clearData();
  virtual void updateImpl();
  virtual void visualizePoints();

  PointCloud* getPrevFrame(void);
  PointCloud* getNextFrame(void);

  // source is this point cloud, target is mesh_model, knn_idx is based on mesh_model
  void searchNearestIdx(MeshModel* mesh_model, std::vector<int>& knn_idx, std::vector<float>& knn_dists);

  // region growing
  void region_growing(std::vector<int>& segment_index, int petal_id);


protected:
  std::string                   filename_;

  std::vector<int>              segment_flags_;
  bool                          segmented_;

};

#endif // POINTCLOUD_H