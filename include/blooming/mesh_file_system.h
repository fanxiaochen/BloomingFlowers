
#ifndef MESH_FILE_SYSTEM_H
#define MESH_FILE_SYSTEM_H

#include <unordered_map>
#include <QHash>

#include <osg/ref_ptr>

#include "file_system_model.h"

class PointCloud;

class MeshFileSystem : public FileSystemModel
{

public:
	MeshFileSystem();
	virtual ~MeshFileSystem();

	QModelIndex setRootPath ( const QString & root_path );
	bool setData(const QModelIndex &index, const QVariant &value, int role);

	bool isShown(const std::string& filename) const;

	/*std::string getPointsFolder(int frame);
	std::string getPointsFilename(int frame);

	osg::ref_ptr<PointCloud> getPointCloud(const std::string& filename);
	osg::ref_ptr<PointCloud> getPointCloud(int frame);

	void updatePointCloud(int frame);*/

	/*void showMeshModel(const std::string& filename);
	void hideMeshModel(const std::string& filename);

	void showMeshModel(const QPersistentModelIndex& index);
	void hideMeshModel(const QPersistentModelIndex& index);*/
};
#endif // FILE_SYSTEM_MODEL_H