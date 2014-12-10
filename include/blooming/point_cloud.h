
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

typedef  pcl::PointXYZRGB  Point;
typedef  pcl::PointCloud<Point>  PclPointCloud;


class PointCloud : public QObject, public Renderable, public PclPointCloud
{
    Q_OBJECT

public:
  PointCloud(void);
  virtual ~PointCloud(void);

  virtual const char* className() const {return "PointCloud";}

  bool open(const std::string& filename);
  void reload(void);

  inline const std::string& getFilename(void) const {return filename_;}
  
  int getFrame(void) const;
  bool isShown(void) const;

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