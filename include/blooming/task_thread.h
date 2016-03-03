#ifndef TRACK_THREAD_H
#define TRACK_THREAD_H

#include <QObject>
#include <QThread>

class TrackingSystem;

class FileSystemModel;
class FlowerViewer;

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


// Motion Transfer
class MotionTransferThread: public QThread
{
public:
    MotionTransferThread(TrackingSystem* tracking_system);
    virtual ~MotionTransferThread();

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

// Boundary Detection
class BoundaryThread: public QThread
{
public:
    BoundaryThread(TrackingSystem* tracking_system);
    virtual ~BoundaryThread();

protected:
    void run();

protected:
    TrackingSystem*	tracking_system_;
};


class FittingErrorThread : public QThread
{
public:
	FittingErrorThread(PointsFileSystem* points_file_system, FlowersViewer* flower_viewer);
	virtual ~FittingErrorThread();

protected:
	void run();

protected:
	PointsFileSystem*	points_file_system_;
	FlowersViewer*		flower_viewer_;
};



// region matching
class RegionMatchingThread: public QThread
{
public:
    RegionMatchingThread(PointCloud* point_cloud, Flower* flower);
    virtual ~RegionMatchingThread();

protected:
    void run();

protected:
    PointCloud* point_cloud_;
    Flower* flower_;
};



// collision detection
class CollisionDetectionThread : public QThread
{
public:
	CollisionDetectionThread( Flower* flower);
	virtual ~CollisionDetectionThread();

protected:
	void run();

protected:
	Flower* flower_;
};

#endif