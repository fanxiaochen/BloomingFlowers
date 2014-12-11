
#include <QDir>
#include <QMutexLocker>

#include <osg/Group>
#include <osgDB/ReadFile>

#include "mesh_model.h"
#include "main_window.h"
#include "point_cloud.h"
#include "osg_viewer_widget.h"
#include "scene_widget.h"
#include "mesh_file_system.h"

MeshFileSystem::MeshFileSystem()
{
    setNameFilterDisables(false);
    QStringList allowed_file_extensions;
    allowed_file_extensions.push_back("*.obj");
    setNameFilters(allowed_file_extensions);

    /*connect(this, SIGNAL(timeToHideAndShowPointCloud(int, int, int, int)), this, SLOT(hideAndShowPointCloud(int, int, int, int)));
    connect(this, SIGNAL(timeToShowPointCloud(int, int)), this, SLOT(showPointCloud(int, int)));*/
}

MeshFileSystem::~MeshFileSystem()
{
}

QModelIndex MeshFileSystem::setRootPath(const QString & root_path)
{
    checked_indexes_.clear();
    mesh_model_map_.clear();

    QModelIndex index = FileSystemModel::setRootPath(root_path);

    return index;
}

bool MeshFileSystem::setData(const QModelIndex &index, const QVariant &value, int role)
{
    bool is_obj = (filePath(index).right(3) == "obj");
    if(role == Qt::CheckStateRole)
    {
        if (is_obj)
        {
            if(value == Qt::Checked)
                showMeshModel(index);
            else
                hideMeshModel(index);
        }

        if(hasChildren(index) == true)
            recursiveCheck(index, value);

        emit dataChanged(index, index);

        return true;
    }

    return FileSystemModel::setData(index, value, role);
}

bool MeshFileSystem::isShown(const std::string& filename) const
{
    QModelIndex index = this->index(filename.c_str());
    return (checked_indexes_.contains(index)) ? (true) : (false);
}


osg::ref_ptr<MeshModel> MeshFileSystem::getMeshModel(const std::string& filename)
{
    QFileInfo fileinfo(filename.c_str());
    if (!fileinfo.exists() || !fileinfo.isFile())
        return NULL;

    osg::ref_ptr<MeshModel> mesh_model = new MeshModel();
    
    if (!mesh_model->load(filename))
    {
        delete mesh_model;
        return NULL;
    }

    return mesh_model;
}

osg::ref_ptr<MeshModel> MeshFileSystem::getMeshModel(QPersistentModelIndex index)
{
    return mesh_model_map_[index];
}


void MeshFileSystem::showMeshModel(const QPersistentModelIndex& index)
{
    checked_indexes_.insert(index);

    MeshModelMap::iterator mesh_model_map_it = mesh_model_map_.find(index);
    if (mesh_model_map_it != mesh_model_map_.end())
        return;

    osg::ref_ptr<MeshModel> mesh_model = getMeshModel(filePath(index).toStdString());

    if (!mesh_model.valid())
        return;

    MainWindow::getInstance()->getSceneWidget()->addSceneChild(mesh_model);
    mesh_model_map_[index] = mesh_model;

    return;
}


void MeshFileSystem::hideMeshModel(const QPersistentModelIndex& index)
{
    checked_indexes_.remove(index);

    MeshModelMap::iterator mesh_model_map_it = mesh_model_map_.find(index);
    if (mesh_model_map_it == mesh_model_map_.end())
        return;

    MainWindow::getInstance()->getSceneWidget()->removeSceneChild(mesh_model_map_it.value().get());
    mesh_model_map_.erase(mesh_model_map_it);

    return;
}

