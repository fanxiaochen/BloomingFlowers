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

class SegmentThread: public QThread
{
public:
    SegmentThread(PointsFileSystem* points_file_system, int frame);
    virtual ~SegmentThread();

protected:
    void run();

protected:
    PointsFileSystem*	points_file_system_;
    int                 frame_;
};


class PropagateSegmentsThread: public QThread
{
public:
    PropagateSegmentsThread(TrackingSystem* tracking_system);
    virtual ~PropagateSegmentsThread();

protected:
    void run();

protected:
    TrackingSystem* tracking_system_;
};

#endif