#ifndef TRACK_THREAD_H
#define TRACK_THREAD_H

#include <QObject>
#include <QThread>

class TrackingSystem;

// EM + ARAP Tracking
class EATrackThread: public QThread
{
public:
    EATrackThread(TrackingSystem* tracking_system);
    virtual ~EATrackThread();

protected:
    void run();

protected:
    TrackingSystem*	tracking_system_;
};


#endif