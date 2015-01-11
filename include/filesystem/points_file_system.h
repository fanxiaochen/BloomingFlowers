
#ifndef POINTS_FILE_SYSTEM_H
#define POINTS_FILE_SYSTEM_H

#include <unordered_map>
#include <QHash>

#include <osg/ref_ptr>

#include "file_system_model.h"

class PointCloud;
class Flower;

class PointsFileSystem : public FileSystemModel
{
  Q_OBJECT

public:
    PointsFileSystem();
    virtual ~PointsFileSystem();

    QModelIndex setRootPath ( const QString & root_path );
    bool setData(const QModelIndex &index, const QVariant &value, int role);

    bool isShown(const std::string& filename) const;

    void getFrameRange(int &start, int &end);
    int getStartFrame(void) const {return start_frame_;}
    int getEndFrame(void) const {return end_frame_;}

    std::string getPointsFolder(int frame);
    std::string getPointsFilename(int frame);

    osg::ref_ptr<PointCloud> getPointCloud(const std::string& filename);
    osg::ref_ptr<PointCloud> getPointCloud(int frame);

    void showPointCloud(const std::string& filename);
    void hidePointCloud(const std::string& filename);

    void showPointCloud(const QPersistentModelIndex& index);
    void hidePointCloud(const QPersistentModelIndex& index);

    void showPointCloud(int frame);
    void hidePointCloud(int frame);

    void updatePointCloud(int frame);


    PointCloud* getDisplayFirstFrame(void);

    void navigateToPreviousFrame();
    void navigateToNextFrame();

    void hideAndShowPointCloud(int hide_frame, int show_frame);

    //void segmentPointCloudByKmeans(int frame);

    //public slots:
    //    void kmeans_segmentation();
private:

    //void limitPointCloudCacheSize(void);
    void computeFrameRange(void);

private:
    typedef QHash<QPersistentModelIndex, osg::ref_ptr<PointCloud> > PointCloudMap;
    typedef std::unordered_map<std::string, osg::ref_ptr<PointCloud> > PointCloudCacheMap;

    PointCloudMap					point_cloud_map_;
    PointCloudCacheMap				point_cloud_cache_map_;

    int								start_frame_;
    int								end_frame_;
};
#endif // FILE_SYSTEM_MODEL_H