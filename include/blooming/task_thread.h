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

// WEM + ARAP Tracking
class WEATrackThread: public QThread
{
public:
    WEATrackThread(TrackingSystem* tracking_system);
    virtual ~WEATrackThread();

protected:
    void run();

protected:
    TrackingSystem*	tracking_system_;
};


// EM + Elastic Tracking
class EETrackThread : public QThread
{
public:
	EETrackThread(TrackingSystem* tracking_system);
	virtual ~EETrackThread();

protected:
	void run();
	
protected:
	TrackingSystem* tracking_system_;
};


// LBS + ARAP Tracking
class LATrackThread: public QThread
{
public:
    LATrackThread(TrackingSystem* tracking_system);
    virtual ~LATrackThread();

protected:
    void run();

protected:
    TrackingSystem*	tracking_system_;
};


// Tip Detection
class TipThread: public QThread
{
public:
    TipThread(TrackingSystem* tracking_system);
    virtual ~TipThread();

protected:
    void run();

protected:
    TrackingSystem*	tracking_system_;
};



#endif