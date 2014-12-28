
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
    p.x = p1.x + p2.x;
    p.y = p1.y + p2.y;
    p.z = p1.z + p2.z;

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

class PointCloud : public QObject, public Renderable, public PclPointCloud
{
public:

    // kmeans_segmentation
    inline const std::vector<int>& getPickedIndices() const { return picked_indices_; }
    inline std::vector<osg::Vec3>& getPickedPoints() { return picked_points_; }
    void pickEvent(int pick_mode, osg::Vec3 position);

private:
    // kmeans_segmentation
    void k_means();
    float distance(const Point& p1, const Point& p2);
    void estimateNormals();
    void computeFlowerPoints();
    void setCenters();
    void updateCenters();
    int determineCluster(const Point& point);
    Point mean_center(const std::vector<int>& ids);
    bool terminal(const std::vector<Point>& cluster_centers, const std::vector<Point>& next_centers);

private:
    // kmeans_segmentation
    std::vector<int>              picked_indices_;
    std::vector<osg::Vec3>        picked_points_;
    std::vector<Point>            cluster_centers_;

public:
  PointCloud(void);
  virtual ~PointCloud(void);

 // virtual const char* className() const {return "PointCloud";}

  bool open(const std::string& filename);
  void reload(void);

  inline const std::string& getFilename(void) const {return filename_;}
  
  int getFrame(void) const;
  bool isShown(void) const;
  
  // using idx vector of this point cloud to build a new cloud
  void reordering(osg::ref_ptr<PointCloud> point_cloud, const std::vector<int>& idx);

  // k-means segmentation
  void kmeans_segmentation();

  // template segmentation with visibility
  void template_segmentation(Flower* flower);

  osg::ref_ptr<PointCloud> getPetalCloud(int id);

  void resetSegmentation();

protected:
  virtual void clearData();
  virtual void updateImpl();
  virtual void visualizePoints();

  PointCloud* getPrevFrame(void);
  PointCloud* getNextFrame(void);

  // source is this point cloud, target is mesh_model, knn_idx is based on mesh_model
  void searchNearestIdx(MeshModel* mesh_model, std::vector<int>& knn_idx, std::vector<float>& knn_dists);


protected:
  std::string                   filename_;

  std::vector<int>              segment_flags_;
  bool                          segmented_;

};

#endif // POINTCLOUD_H