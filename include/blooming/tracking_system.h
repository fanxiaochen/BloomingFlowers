#ifndef TRACKING_SYSTEM_H
#define TRACKING_SYSTEM_H

#include <QObject>

class PointCloud;
class PointsFileSystem;
class MeshFileSystem;

class TrackingSystem: public QObject
{
	Q_OBJECT

public:
	TrackingSystem(PointsFileSystem* points_file_system, MeshFileSystem* mesh_file_system);
	~TrackingSystem();
	
public slots:
	void track();

private:
	void cpd_registration(const PointCloud& tracked_frame, PointCloud& tracking_template);

private:
	PointsFileSystem*	points_file_system_;
	MeshFileSystem*		mesh_file_system_;
};

#endif