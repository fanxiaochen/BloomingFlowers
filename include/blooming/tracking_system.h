#ifndef TRACKING_SYSTEM_H
#define TRACKING_SYSTEM_H

#include <QObject>

class PointCloud;
class MeshModel;
class PointsFileSystem;
class MeshFileSystem;

class TrackingSystem: public QObject
{
	Q_OBJECT

public:
	friend class PointsTrackThread;
    friend class MeshTrackThread;

	TrackingSystem(PointsFileSystem* points_file_system, MeshFileSystem* mesh_file_system);
	~TrackingSystem();
	
public slots:
	void pointcloud_tracking();
    void mesh_tracking();

private:
	void cpd_registration(const PointCloud& tracked_frame, PointCloud& tracking_template);
	void cpd_registration(const PointCloud& tracked_frame, MeshModel& tracking_template);

private:
	PointsFileSystem*	points_file_system_;
	MeshFileSystem*		mesh_file_system_;
};

#endif