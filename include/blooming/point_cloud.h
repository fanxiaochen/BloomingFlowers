
#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <QObject>
#include <QColor>
#include <osg/Array>
#include <unordered_set>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "renderable.h"
   
typedef  pcl::PointXYZRGBNormal        Point;
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
class TrajectoryModel;

class PointCloud : public QObject, public Renderable, public PclPointCloud
{
public:
    typedef std::pair<std::vector<int>, osg::ref_ptr<PointCloud>> MatchRegion;
    typedef std::vector<MatchRegion> MatchRegions;

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
  bool flower_segmentation(Flower* flower);

  // for data-driven solver
  // if couldn't be segmented, use trajectory model to predict
  // or use the segmentation result
  void fitting_region(Flower* flower, TrajectoryModel* traj_model);

  // trajectory prediction
  void trajectory_prediction(TrajectoryModel* traj_model);

  // radius search from predict point
  void prediction_search(const Point& predict_point, double radius, int region_id);

  // predict new positions for trajectories
  void prediction_search(std::vector<int>& petal_order, const Point& origin_point, const Point& direction, double radius, int region_id);

  osg::ref_ptr<PointCloud> getPetalCloud(int id);

  osg::ref_ptr<PointCloud> getSamplingPetalCloud(int id, int radio);

  std::vector<int>& getSegmentFlags() { return segment_flags_; }
  
  std::vector<int>& getTips() { return tip_indices_; }
  std::vector<int>& getBoundary() { return boundary_indices_; }
  std::vector<std::vector<int>>& getBoundarySegments() { return boundary_segments_; }
  std::vector<std::vector<int>>& getTipsSegments() { return tips_segments_; }

  osg::ref_ptr<PointCloud> getBoundary(int id);
  osg::ref_ptr<PointCloud> getTips(int id);

  bool& getShowBoundary() { return show_boundary_; }
  bool& getShowTips() { return show_tips_; }

  void region_matching(Flower* flower);
  void region_completion(Flower* flower);
  osg::ref_ptr<PointCloud> getFittingCloud(int id);
  std::vector<int> getFittingMesh(int id);

  void region_segmentation(Flower* flower);

  bool boundary_segmentation(Flower* flower);

  void indicateSegmentFlags(Flower* flower);

  bool tip_segmentation(Flower* flower);
  void tip_matching(Flower* flower, int id);
  Point searchTip(Point t, PointCloud* boundary);
  void tip_region(Point t, Point k, Flower* flower, int id);
  void searchCloudTips(Point k, int id);
  void searchPetalTips(Point t, Flower* flower, int id);

  // source is this point cloud, target is mesh_model, knn_idx is based on mesh_model
  void searchNearestIdx(MeshModel* mesh_model, std::vector<int>& knn_idx, std::vector<float>& knn_dists);

  // source is mesh model target is this point cloud, knn_idx is based on point cloud
  void searchNearestIdx(MeshModel* mesh_model, std::vector<std::vector<int>>& knn_idx, int K);

  void buildSelfKdtree();

  void setClosureCloud(const std::string& closure_cloud);
  osg::ref_ptr<PointCloud> getClosureCloud(); 

  pcl::KdTreeFLANN<pcl::PointXYZ>& getSelfKdtree() { return kdtree_; }

  void savePovrayFiles(const QString& povray_files);
  void saveLightFile(const QString& light_filename);
  void saveTextureFile(const QString& texture_filename);
  void saveCameraFile(const QString& camera_filename);
  void saveDataFile(const QString& data_filename);

protected:
  virtual void clearData();
  virtual void updateImpl();
  virtual void visualizePoints();

  PointCloud* getPrevFrame(void);
  PointCloud* getNextFrame(void);

  // region growing
  void region_growing(std::vector<int>& segment_index, int petal_id);

  // gaussian distribution
  double gaussian(int m_id, int c_id, MeshModel* petal);

  bool isInRegion(int knn_idx, int region_id, Flower* flower);


protected:
  std::string                   filename_;

  std::vector<int>              segment_flags_;
  bool                          segmented_;
  int                           segment_number_;

  std::vector<int>              color_flags_;

  MatchRegions                  match_regions_;

  Eigen::MatrixXd               P_;

  std::vector<int>              tip_indices_;
  std::vector<int>              boundary_indices_;

  std::vector<std::vector<int>> boundary_segments_;
  std::vector<std::vector<int>> tips_segments_;

  bool                          show_boundary_;
  bool                          show_tips_;
  bool                          show_probs_;;

  pcl::KdTreeFLANN<pcl::PointXYZ>     kdtree_;

  osg::ref_ptr<PointCloud>            closure_cloud_;

};

#endif // POINTCLOUD_H