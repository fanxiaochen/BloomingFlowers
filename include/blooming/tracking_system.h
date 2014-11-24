#ifndef TRACKING_SYSTEM_H
#define TRACKING_SYSTEM_H

#include <QObject>
#include <vector>

class PointCloud;
class MeshModel;
class PointsFileSystem;
class MeshFileSystem;
class Trajectories;

class TrackingSystem: public QObject
{
	Q_OBJECT

public:
	TrackingSystem(PointsFileSystem* points_file_system, MeshFileSystem* mesh_file_system);
	~TrackingSystem();

    inline PointsFileSystem* getPointsFileSystem(){ return points_file_system_; }
    inline MeshFileSystem* getMeshFileSystem(){ return mesh_file_system_; }
    inline Trajectories* getTrajectories(){ return trajectories_; }
	
public slots:
	void pointcloud_tracking();
    void mesh_tracking();

    void buildTrajectories();
    void clusterTrajectories();

public:
	void cpd_registration(const PointCloud& tracked_frame, PointCloud& tracking_template);
	void cpd_registration(const PointCloud& tracked_frame, MeshModel& tracking_template);
    void cpd_registration(const PointCloud& source_frame, const PointCloud& target_frame, 
        const std::vector<int>& src_idx, std::vector<int>& tar_idx);

private:
	PointsFileSystem*	points_file_system_;
	MeshFileSystem*		mesh_file_system_;

    Trajectories*       trajectories_;
};

#endif