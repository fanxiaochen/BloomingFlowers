#ifndef TRACK_THREAD_H
#define TRACK_THREAD_H

#include <QObject>
#include <QThread>

class TrackingSystem;

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


#endif