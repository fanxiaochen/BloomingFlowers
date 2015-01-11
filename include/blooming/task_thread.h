#ifndef TRACK_THREAD_H
#define TRACK_THREAD_H

#include <QObject>
#include <QThread>

class TrackingSystem;

class EMTrackThread: public QThread
{
public:
    EMTrackThread(TrackingSystem* tracking_system);
    virtual ~EMTrackThread();

protected:
    void run();

protected:
    TrackingSystem*	tracking_system_;
};


#endif