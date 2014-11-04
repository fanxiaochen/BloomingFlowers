#ifndef TRACKING_SYSTEM_H
#define TRACKING_SYSTEM_H

#include <QObject>
#include <QThread>

class PointCloud;
class MeshModel;
class PointsFileSystem;
class MeshFileSystem;
class TaskController;

class TrackingSystem: public QObject
{
	Q_OBJECT

public:
	friend class TrackThread;

	TrackingSystem(PointsFileSystem* points_file_system, MeshFileSystem* mesh_file_system);
	~TrackingSystem();
	
public slots:
	void track();

private:
	void cpd_registration(const PointCloud& tracked_frame, PointCloud& tracking_template);
	void cpd_registration(const PointCloud& tracked_frame, MeshModel& tracking_template);

private:
	PointsFileSystem*	points_file_system_;
	MeshFileSystem*		mesh_file_system_;

	TrackThread*		track_thread_;
};


class TrackThread: public QThread
{
	Q_OBJECT
public:
	TrackThread(TrackingSystem* tracking_system);
	virtual ~TrackThread();

protected:
	void run();

protected:
	TrackingSystem*	tracking_system_;
};

#endif