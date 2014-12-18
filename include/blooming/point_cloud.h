
#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <QObject>
#include <QColor>
#include <osg/Array>
#include <unordered_set>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "renderable.h"

typedef  pcl::PointXYZRGB  Point;
typedef  pcl::PointCloud<Point>  PclPointCloud;


class PointCloud : public QObject, virtual public Renderable, public PclPointCloud
{
public:
    struct FlowerPoint
    {
        int _label;
        Point _pt;

        FlowerPoint()
        {
            _pt.x = 0;
            _pt.y = 0;
            _pt.z = 0;

            _label = -1;
        }

        FlowerPoint& operator + (const FlowerPoint& point)
        {
            this->_pt.x = this->_pt.x + point._pt.x;
            this->_pt.y = this->_pt.y + point._pt.y;
            this->_pt.z = this->_pt.z + point._pt.z;

            return *this;
        }

        FlowerPoint& operator - (const FlowerPoint& point)
        {
            this->_pt.x = this->_pt.x - point._pt.x;
            this->_pt.y = this->_pt.y - point._pt.y;
            this->_pt.z = this->_pt.z - point._pt.z;

            return *this;
        }

        FlowerPoint& operator * (float mul)
        {
            this->_pt.x = this->_pt.x * mul;
            this->_pt.y = this->_pt.y * mul;
            this->_pt.z = this->_pt.z * mul;

            return *this;
        }

        FlowerPoint& operator / (float div)
        {
            this->_pt.x = this->_pt.x / div;
            this->_pt.y = this->_pt.y / div;
            this->_pt.z = this->_pt.z / div;

            return *this;
        }

        FlowerPoint& operator = (const FlowerPoint& point)
        {
            this->_pt.x = point._pt.x;
            this->_pt.y = point._pt.y;
            this->_pt.z = point._pt.z;

            this->_label = point._label;

            return *this;
        }

        bool operator == (const FlowerPoint& point)
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

    // segmentation
    inline std::vector<FlowerPoint>& getFlowerPoints() { return flower_points_; }
    inline const std::vector<int>& getPickedIndices() const { return picked_indices_; }
    inline std::vector<osg::Vec3>& getPickedPoints() { return picked_points_; }
    void pickEvent(int pick_mode, osg::Vec3 position);
    void petal_segmentation();
    void resetSegmentation();

private:
    // segmentation
    void k_means();
    float distance(const FlowerPoint& p1, const FlowerPoint& p2);
    void estimateNormals();
    void computeFlowerPoints();
    void setCenters();
    void updateCenters();
    int determineCluster(const FlowerPoint& point);
    FlowerPoint mean_center(const std::vector<int>& ids);
    bool terminal(const std::vector<FlowerPoint>& cluster_centers, const std::vector<FlowerPoint>& next_centers);

private:
    // segmentation
    std::vector<FlowerPoint>      flower_points_;

    std::vector<int>              picked_indices_;
    std::vector<osg::Vec3>        picked_points_;
    std::vector<FlowerPoint>      cluster_centers_;

    bool                          segmented_;


public:
  PointCloud(void);
  virtual ~PointCloud(void);

 // virtual const char* className() const {return "PointCloud";}

  bool open(const std::string& filename);
  void reload(void);

  inline const std::string& getFilename(void) const {return filename_;}
  
  int getFrame(void) const;
  bool isShown(void) const;

  void reordering(osg::ref_ptr<PointCloud> point_cloud, const std::vector<int>& idx);

protected:
  virtual void clearData();
  virtual void updateImpl();
  virtual void visualizePoints();

  PointCloud* getPrevFrame(void);
  PointCloud* getNextFrame(void);

protected:
  std::string                    filename_;
};

#endif // POINTCLOUD_H