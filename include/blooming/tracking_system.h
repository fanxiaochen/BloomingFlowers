#ifndef TRACKING_SYSTEM_H
#define TRACKING_SYSTEM_H

class PointCloud;
class PointsFileSystem;
class MeshFileSystem;

class TrackingSystem
{
public:
	TrackingSystem(PointsFileSystem* points_file_system, MeshFileSystem* mesh_file_system);
	~TrackingSystem();
	
	void track();

private:
	void cpd_registration(PointCloud* tracking_template, PointCloud* tracked_frame);

private:
	PointsFileSystem*	points_file_system_;
	MeshFileSystem*		mesh_file_system_;
};

#endif