#ifndef TRACKING_SYSTEM_H
#define TRACKING_SYSTEM_H

class PointCloud;
class FileSystemModel;

class TrackingSystem
{
public:
	TrackingSystem(FileSystemModel* points_file_system, FileSystemModel* mesh_file_system);
	~TrackingSystem();
	
	void track();

private:
	FileSystemModel*	points_file_system_;
	FileSystemModel*	mesh_file_system_;
};

#endif