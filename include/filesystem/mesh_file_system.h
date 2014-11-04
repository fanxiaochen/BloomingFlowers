
#ifndef MESH_FILE_SYSTEM_H
#define MESH_FILE_SYSTEM_H

#include <unordered_map>
#include <QHash>

#include <osg/ref_ptr>

#include "file_system_model.h"

class MeshModel;

class MeshFileSystem : public FileSystemModel
{

public:
	MeshFileSystem();
	virtual ~MeshFileSystem();

	QModelIndex setRootPath ( const QString & root_path );
	bool setData(const QModelIndex &index, const QVariant &value, int role);

	bool isShown(const std::string& filename) const;

	osg::ref_ptr<MeshModel> getMeshModel(const std::string& filename);
	osg::ref_ptr<MeshModel> getMeshModel(QPersistentModelIndex index);

	void showMeshModel(const std::string& filename);
	void hideMeshModel(const std::string& filename);

	void showMeshModel(const QPersistentModelIndex& index);
	void hideMeshModel(const QPersistentModelIndex& index);

private:
	typedef QHash<QPersistentModelIndex, osg::ref_ptr<MeshModel> > MeshModelMap;
	
	MeshModelMap	mesh_model_map_;
};
#endif 