#ifndef TRACKING_SYSTEM_H
#define TRACKING_SYSTEM_H

#include <QObject>
#include <vector>

#include "flower.h"

class PointCloud;
class MeshModel;
class PointsFileSystem;
class MeshFileSystem;
class Trajectories;

class TrackingSystem: public QObject
{
    Q_OBJECT

public:
    TrackingSystem(PointsFileSystem* point_file_system);
    TrackingSystem(PointsFileSystem* points_file_system, MeshFileSystem* mesh_file_system);
    ~TrackingSystem();

    inline PointsFileSystem* getPointsFileSystem(){ return points_file_system_; }
    inline MeshFileSystem* getMeshFileSystem(){ return mesh_file_system_; }
    inline Trajectories* getTrajectories(){ return trajectories_; }

    inline Flowers* getFlowers(){ return flowers_; }

    inline void setStartFrame(int start_frame) { start_frame_ = start_frame; }
    inline void setEndFrame(int end_frame) { end_frame_ = end_frame; }
    inline void setKeyFrame(int key_frame) { key_frame_ = key_frame; }

    inline int getStartFrame() { return start_frame_; }
    inline int getEndFrame() { return end_frame_; }
    inline int getKeyFrame() { return key_frame_; }

    public slots:
    void pointcloud_tracking();
    void mesh_tracking();

    void buildTrajectories();
    void clusterTrajectories();

    void propagateSegments();

    void flower_tracking();



public:
    void cpd_registration(const PointCloud& tracked_frame, PointCloud& tracking_template);
    void cpd_registration(const PointCloud& tracked_frame, MeshModel& tracking_template);
    void cpd_registration(const PointCloud& source_frame, const PointCloud& target_frame, 
        const std::vector<int>& src_idx, std::vector<int>& tar_idx);
    void cpd_registration(const PointCloud& tracked_frame, Flower& tracking_template);


private:
    PointsFileSystem*   points_file_system_;
    MeshFileSystem*     mesh_file_system_;

    Trajectories*       trajectories_;

    int                 start_frame_;
    int                 end_frame_;
    int                 key_frame_;

    Flowers*             flowers_;
};

#endif