#ifndef TRACKING_SYSTEM_H
#define TRACKING_SYSTEM_H

#include <QObject>
#include <vector>

#include "flower.h"

class PointCloud;
class MeshModel;
class PointsFileSystem;
class MeshFileSystem;

class TrackingSystem: public QObject
{
    Q_OBJECT

public:
    TrackingSystem(PointsFileSystem* point_file_system);
    TrackingSystem(PointsFileSystem* points_file_system, MeshFileSystem* mesh_file_system);
    ~TrackingSystem();

    inline PointsFileSystem* getPointsFileSystem(){ return points_file_system_; }
    inline MeshFileSystem* getMeshFileSystem(){ return mesh_file_system_; }

    inline void setStartFrame(int start_frame) { start_frame_ = start_frame; }
    inline void setEndFrame(int end_frame) { end_frame_ = end_frame; }
    inline void setKeyFrame(int key_frame) { key_frame_ = key_frame; }

    inline int getStartFrame() { return start_frame_; }
    inline int getEndFrame() { return end_frame_; }
    inline int getKeyFrame() { return key_frame_; }

    public slots:

    void em_arap();
	void em_elastic();
    void wem_arap();
    void lbs_arap();

    void detectTip();
    void detectBoundary();


public:
    void ea_registration(PointCloud& tracked_frame, Flower& tracking_template);
	void ee_registration(PointCloud& tracked_frame, Flower& tracking_template);
    void wea_registration(PointCloud& tracked_frame, Flower& tracking_template);
    void la_registration(PointCloud& tracked_frame, Flower& tracking_template);

private:
    PointsFileSystem*   points_file_system_;
    MeshFileSystem*     mesh_file_system_;

    int                 start_frame_;
    int                 end_frame_;
    int                 key_frame_;

};

#endif