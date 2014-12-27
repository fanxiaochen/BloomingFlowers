#ifndef TRACK_THREAD_H
#define TRACK_THREAD_H

#include <QObject>
#include <QThread>

class TrackingSystem;
class Trajectories;
class PointsFileSystem;

class PointsTrackThread: public QThread
{
public:
    PointsTrackThread(TrackingSystem* tracking_system);
    virtual ~PointsTrackThread();

protected:
    void run();

protected:
    TrackingSystem*	tracking_system_;
};


class MeshTrackThread: public QThread
{
public:
    MeshTrackThread(TrackingSystem* tracking_system);
    virtual ~MeshTrackThread();

protected:
    void run();

protected:
    TrackingSystem*	tracking_system_;
};

class PetalTrackThread: public QThread
{
public:
    PetalTrackThread(TrackingSystem* tracking_system);
    virtual ~PetalTrackThread();

protected:
    void run();

protected:
    TrackingSystem*	tracking_system_;
};

class FlowerTrackThread: public QThread
{
public:
    FlowerTrackThread(TrackingSystem* tracking_system);
    virtual ~FlowerTrackThread();

protected:
    void run();

protected:
    TrackingSystem*	tracking_system_;
};

class TrajectoryTrackThread: public QThread
{
public:
    TrajectoryTrackThread(TrackingSystem* tracking_system);
    virtual ~TrajectoryTrackThread();

protected:
    void run();

protected:
    TrackingSystem*	tracking_system_;
};


class TrajClusteringThread: public QThread
{
public:
    TrajClusteringThread(Trajectories* trajectories);
    virtual ~TrajClusteringThread();

protected:
    void run();

protected:
    Trajectories*	trajectories_;
};

class KmeansSegmentThread: public QThread
{
public:
    KmeansSegmentThread(PointsFileSystem* points_file_system, int frame);
    virtual ~KmeansSegmentThread();

protected:
    void run();

protected:
    PointsFileSystem*	points_file_system_;
    int                 frame_;
};

class TemplateSegmentThread: public QThread
{
public:
    TemplateSegmentThread(TrackingSystem* tracking_system);
    virtual ~TemplateSegmentThread();

protected:
    void run();

protected:
    TrackingSystem*  tracking_system_;
};


#endif