#ifndef TRACK_THREAD_H
#define TRACK_THREAD_H

#include <QObject>
#include <QThread>

class TrackingSystem;
class Trajectories;

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


#endif