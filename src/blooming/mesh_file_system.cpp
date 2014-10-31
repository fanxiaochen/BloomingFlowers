#include <QDir>
#include <QMutexLocker>

#include <osg/Group>
#include <osgDB/ReadFile>

#include "main_window.h"
#include "point_cloud.h"
#include "osg_viewer_widget.h"
#include "mesh_file_system.h"
#include "scene_widget.h"


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

	QModelIndex index = FileSystemModel::setRootPath(root_path);
	/*computeFrameRange();

	if (start_frame_ != -1)
	{
		if (getPointCloud(start_frame_) != NULL)
			showPointCloud(start_frame_);
	}

	MainWindow::getInstance()->getSceneWidget()->centerScene();*/

	return index;
}

bool MeshFileSystem::setData(const QModelIndex &index, const QVariant &value, int role)
{
	/*bool is_point_cloud = (filePath(index).right(3) == "obj");
	if(role == Qt::CheckStateRole)
	{
		if (is_point_cloud)
		{
			if(value == Qt::Checked)
				showPointCloud(index);
			else
				hidePointCloud(index);
		}

		if(hasChildren(index) == true)
			recursiveCheck(index, value);

		emit dataChanged(index, index);

		return true;
	}*/

	return FileSystemModel::setData(index, value, role);
}

bool MeshFileSystem::isShown(const std::string& filename) const
{
	QModelIndex index = this->index(filename.c_str());
	return (checked_indexes_.contains(index)) ? (true) : (false);
}


//osg::ref_ptr<PointCloud> MeshFileSystem::getPointCloud(const std::string& filename)
//{
//	QMutexLocker locker(&mutex_);
//
//	//limitPointCloudCacheSize();
//
//	QFileInfo fileinfo(filename.c_str());
//	if (!fileinfo.exists() || !fileinfo.isFile())
//		return NULL;
//
//	PointCloudCacheMap::iterator it = point_cloud_cache_map_.find(filename);
//	if (it != point_cloud_cache_map_.end())
//		return it->second.get();
//
//	osg::ref_ptr<PointCloud> point_cloud(new PointCloud());
//	if (!point_cloud->open(filename))
//		return NULL;
//
//	if (point_cloud->thread() != qApp->thread())
//		point_cloud->moveToThread(qApp->thread());
//
//	point_cloud_cache_map_[filename] = point_cloud;
//
//	return point_cloud;
//}
//
//osg::ref_ptr<PointCloud> MeshFileSystem::getPointCloud(int frame)
//{
//	return getPointCloud(getPointsFilename(frame));
//}
//
//
//std::string MeshFileSystem::getPointsFolder(int frame)
//{
//	QModelIndex root_index = index(rootPath());
//
//	int start, end;
//	getFrameRange(start, end);
//	if (start < 0 || end < 0)
//		return std::string();
//
//	std::string folder;
//
//	QString root_path = rootPath();
//	if (root_path.contains("frame_"))
//	{
//		folder =  root_path.toStdString();
//	}
//	else if (root_path.contains("points"))
//	{
//		// not work ??
//		/*std::cout << root_path.toStdString() << std::endl;
//		QModelIndex frame_index = index(frame-start, 0, root_index);
//		folder = filePath(frame_index).toStdString();*/
//
//		QString frame_name(QString("frame_%1").arg(frame, 5, 10, QChar('0')));
//		QStringList root_entries = QDir(root_path).entryList();
//		for (QStringList::const_iterator root_entries_it = root_entries.begin();
//			root_entries_it != root_entries.end(); ++ root_entries_it)
//		{
//			if (root_entries_it->compare(frame_name) == 0)
//			{
//				folder = (root_path + QString("/%1").arg(*root_entries_it)).toStdString();
//				break;
//			}
//		}
//	}
//
//	return folder;
//}
//
//std::string MeshFileSystem::getPointsFilename(int frame)
//{
//	std::string folder = getPointsFolder(frame);
//	if (folder.empty())
//		return folder;
//
//	return folder+"/points.pcd";
//}

//void MeshFileSystem::showPointCloud(int frame)
//{
//	showPointCloud(getPointsFilename(frame));
//}
//
//void MeshFileSystem::showPointCloud(const std::string& filename)
//{
//	QModelIndex index = this->index(QString(filename.c_str()));
//	if (!index.isValid())
//		return;
//
//	showPointCloud(index);
//
//	return;
//}
//
//void MeshFileSystem::showPointCloud(const QPersistentModelIndex& index)
//{
//	checked_indexes_.insert(index);
//
//	osg::ref_ptr<PointCloud> point_cloud(getPointCloud(filePath(index).toStdString()));
//	if (!point_cloud.valid())
//		return;
//
//	PointCloudMap::iterator point_cloud_map_it = point_cloud_map_.find(index);
//	if (point_cloud_map_it != point_cloud_map_.end())
//		return;
//
//	MainWindow::getInstance()->getSceneWidget()->addSceneChild(point_cloud);
//	point_cloud_map_[index] = point_cloud;
//
//	return;
//}
//
//void MeshFileSystem::hidePointCloud(int frame)
//{
//	hidePointCloud(getPointsFilename(frame));
//}
//
//void MeshFileSystem::hidePointCloud(const std::string& filename)
//{
//	QModelIndex index = this->index(QString(filename.c_str()));
//	if (!index.isValid())
//		return;
//
//	hidePointCloud(index);
//}
//
//
//void MeshFileSystem::hidePointCloud(const QPersistentModelIndex& index)
//{
//	checked_indexes_.remove(index);
//
//	PointCloudMap::iterator point_cloud_map_it = point_cloud_map_.find(index);
//	if (point_cloud_map_it == point_cloud_map_.end())
//		return;
//
//	MainWindow::getInstance()->getSceneWidget()->removeSceneChild(point_cloud_map_it.value().get());
//	point_cloud_map_.erase(point_cloud_map_it);
//
//	return;
//}
//
//void MeshFileSystem::updatePointCloud(int frame)
//{
//	std::string filename = getPointsFilename(frame);
//	if (point_cloud_cache_map_.find(filename) == point_cloud_cache_map_.end())
//		return;
//
//	getPointCloud(frame)->reload();
//
//	return;
//}

