
#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <QObject>
#include <QColor>
#include <osg/Array>
#include <unordered_set>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "renderable.h"
//#include "petal_cloud.h"

typedef	 pcl::PointXYZRGB  Point;
typedef	 pcl::PointCloud<Point>  PclPointCloud;


class PointCloud : public QObject, public Renderable, public PclPointCloud
{
    Q_OBJECT

public:

    struct ClusterPoint
    {
        int _label;
        Point _pt;
        osg::Vec3 _mv;

        ClusterPoint()
        {
            _pt.x = 0;
            _pt.y = 0;
            _pt.z = 0;

            _mv.x() = 0;
            _mv.y() = 0;
            _mv.z() = 0;

            _label = -1;
        }

        ClusterPoint& operator + (const ClusterPoint& point)
        {
            this->_mv = this->_mv + point._mv;
            this->_pt.x = this->_pt.x + point._pt.x;
            this->_pt.y = this->_pt.y + point._pt.y;
            this->_pt.z = this->_pt.z + point._pt.z;

            return *this;
        }

        ClusterPoint& operator / (float div)
        {
            this->_mv = this->_mv / div;
            this->_pt.x = this->_pt.x / div;
            this->_pt.y = this->_pt.y / div;
            this->_pt.z = this->_pt.z / div;

            return *this;
        }

        bool operator == (const ClusterPoint& point)
        {
            const float eps = 1e-3;

            if (fabs(this->_pt.x - point._pt.x) < eps&&
                fabs(this->_pt.y - point._pt.y) < eps&&
                fabs(this->_pt.z - point._pt.z) < eps)
                return true;
            else
                return false;
        }
    };

public:
  PointCloud(void);
  virtual ~PointCloud(void);

  virtual const char* className() const {return "PointCloud";}

  bool open(const std::string& filename);
  void reload(void);

  inline const std::string& getFilename(void) const {return filename_;}
  inline const std::vector<int>& getPickedIndices() const { return picked_indices_; }
  inline std::vector<osg::Vec3>& getPickedPoints() { return picked_points_; }

  int getFrame(void) const;
  bool isShown(void) const;

  virtual void pickEvent(int pick_mode, osg::Vec3 position);

  void petal_segmentation();

protected:
  virtual void clearData();
  virtual void updateImpl();

  void visualizePoints();

  PointCloud* getPrevFrame(void);
  PointCloud* getNextFrame(void);

  void k_means();
  float distance(const ClusterPoint& p1, const ClusterPoint& p2);
  void estimateNormals();
  void computeClusterPoints();
  void setCenters();
  void updateCenters();
  int determineCluster(const ClusterPoint& point);
  ClusterPoint mean_center(const std::vector<int>& ids);
  bool terminal(const std::vector<ClusterPoint>& cluster_centers, const std::vector<ClusterPoint>& next_centers);

public:
    inline std::vector<ClusterPoint>& getClusterPoints() { return cluster_points_; }

protected:
  std::string                    filename_;

  std::vector<int>               picked_indices_;
  std::vector<osg::Vec3>         picked_points_;

  std::vector<ClusterPoint>      cluster_centers_;
  std::vector<ClusterPoint>      cluster_points_;

  bool                           activated_;
};

#endif // POINTCLOUD_H