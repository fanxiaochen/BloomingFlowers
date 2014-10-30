#include <QDir>
#include <QColor>
#include <QMutexLocker>

#include <osg/Group>
#include <osgDB/ReadFile>

#include "main_window.h"
#include "point_cloud.h"
#include "osg_viewer_widget.h"
#include "file_system_model.h"


FileSystemModel::FileSystemModel()
	:start_frame_(-1),
	end_frame_(-1)
{
	setNameFilterDisables(false);
	QStringList allowed_file_extensions;
	allowed_file_extensions.push_back("*.pcd");
	allowed_file_extensions.push_back("*.obj");
	setNameFilters(allowed_file_extensions);

	/*connect(this, SIGNAL(timeToHideAndShowPointCloud(int, int, int, int)), this, SLOT(hideAndShowPointCloud(int, int, int, int)));
	connect(this, SIGNAL(timeToShowPointCloud(int, int)), this, SLOT(showPointCloud(int, int)));*/
}

FileSystemModel::~FileSystemModel()
{
}

Qt::ItemFlags FileSystemModel::flags(const QModelIndex &index) const
{
	return QFileSystemModel::flags(index) | Qt::ItemIsUserCheckable;
}

QVariant FileSystemModel::data(const QModelIndex &index, int role) const
{
	if(role == Qt::CheckStateRole)
		return computeCheckState(index);
     
	return QFileSystemModel::data(index, role); 
}

Qt::CheckState FileSystemModel::computeCheckState(const QModelIndex &index) const
{
	if(!hasChildren(index))
		return (checked_indexes_.contains(index)) ? (Qt::Checked) : (Qt::Unchecked);

	bool all_checked = true;
	bool all_unchecked = true;
	for(int i = 0, i_end = rowCount(index); i < i_end; i ++)
	{
		QModelIndex child = QFileSystemModel::index(i, 0, index);
		Qt::CheckState check_state = computeCheckState(child);
		if (check_state == Qt::PartiallyChecked)
			return check_state;

		if (check_state == Qt::Checked)
			all_unchecked = false;
		if (check_state == Qt::Unchecked)
			all_checked = false;

		if (!all_checked && !all_unchecked)
			return Qt::PartiallyChecked;
	}

	if (all_unchecked)
		return Qt::Unchecked;

	return Qt::Checked;
}

bool FileSystemModel::recursiveCheck(const QModelIndex &index, const QVariant &value)
{
	if(!hasChildren(index))
		return false;

	for(int i = 0, i_end = rowCount(index); i < i_end; i ++)
	{
		QModelIndex child = QFileSystemModel::index(i, 0, index);
		setData(child, value, Qt::CheckStateRole);
	}

	return true;
}

bool FileSystemModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
	bool is_point_cloud = (filePath(index).right(3) == "pcd");
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
	}

	return QFileSystemModel::setData(index, value, role);
}

bool FileSystemModel::isShown(const std::string& filename) const
{
	QModelIndex index = this->index(filename.c_str());
	return (checked_indexes_.contains(index)) ? (true) : (false);
}

QModelIndex FileSystemModel::setRootPath(const QString & root_path)
{
	point_cloud_cache_map_.clear();
	point_cloud_map_.clear();
	checked_indexes_.clear();

	QModelIndex index = QFileSystemModel::setRootPath(root_path);
	computeFrameRange();

	if (start_frame_ != -1)
	{
		if (getPointCloud(start_frame_) != NULL)
			showPointCloud(start_frame_);
	}

	//MainWindow::getInstance()->getOSGViewerWidget()->centerScene();

	return index;
}



//void FileSystemModel::limitPointCloudCacheSize(void)
//{
//	size_t threshold = 32;
//
//	if (point_cloud_cache_map_.size() <= threshold)
//	return;
//
//	std::set<osg::ref_ptr<PointCloud> > in_use_clouds;
//	std::vector<osg::ref_ptr<PointCloud> > freeable_clouds;
//
//	for (PointCloudMap::const_iterator it = point_cloud_map_.begin(); it != point_cloud_map_.end(); ++ it)
//	in_use_clouds.insert(*it);
//
//	for (PointCloudCacheMap::const_iterator it = point_cloud_cache_map_.begin(); it != point_cloud_cache_map_.end(); ++ it)
//	if (in_use_clouds.find(it->second) == in_use_clouds.end() && it->second->referenceCount() == 1)
//		freeable_clouds.push_back(it->second);
//
//	for (size_t i = 0, i_end = freeable_clouds.size(); i < i_end; ++ i)
//	point_cloud_cache_map_.erase(freeable_clouds[i]->getFilename());
//
//	return;
//}


static void extractStartEndFrame(const QStringList& entries, int& start_frame, int& end_frame)
{
	start_frame = std::numeric_limits<int>::max();
	end_frame = std::numeric_limits<int>::min();

	for (QStringList::const_iterator entries_it = entries.begin();
	entries_it != entries.end(); ++ entries_it)
	{
	if (!entries_it->contains("frame_"))
		continue;

	int index = entries_it->right(4).toInt();
	if (start_frame > index)
		start_frame = index;
	if (end_frame < index)
		end_frame = index;
	}

	return;
}

void FileSystemModel::computeFrameRange(void)
{
	start_frame_ = end_frame_ = -1;

	QString root_path = rootPath();
	QModelIndex root_index = index(root_path);

	if (root_path.contains("frame_")) {
		start_frame_ = end_frame_ = root_path.right(4).toInt();
		return;
	}

	if (root_path.compare("points") == 0)
	{
		QStringList points_entries = QDir(root_path).entryList();
		extractStartEndFrame(points_entries, start_frame_, end_frame_);
		return;
	}

	return;
}

void FileSystemModel::getFrameRange(int &start, int &end)
{
	start = start_frame_;
	end = end_frame_;
}

osg::ref_ptr<PointCloud> FileSystemModel::getPointCloud(const std::string& filename)
{
	QMutexLocker locker(&mutex_);

	//limitPointCloudCacheSize();

	QFileInfo fileinfo(filename.c_str());
	if (!fileinfo.exists() || !fileinfo.isFile())
		return NULL;

	PointCloudCacheMap::iterator it = point_cloud_cache_map_.find(filename);
	if (it != point_cloud_cache_map_.end())
		return it->second.get();

	osg::ref_ptr<PointCloud> point_cloud(new PointCloud());
	if (!point_cloud->open(filename))
		return NULL;

	if (point_cloud->thread() != qApp->thread())
		point_cloud->moveToThread(qApp->thread());

	point_cloud_cache_map_[filename] = point_cloud;

	return point_cloud;
}

osg::ref_ptr<PointCloud> FileSystemModel::getPointCloud(int frame)
{
	return getPointCloud(getPointsFilename(frame));
}


std::string FileSystemModel::getPointsFolder(int frame)
{
	QModelIndex root_index = index(rootPath());

	int start, end;
	getFrameRange(start, end);
	if (start < 0 || end < 0)
		return std::string();

	std::string folder;

	QString root_path = rootPath();
	if (root_path.contains("frame_"))
	{
		folder =  root_path.toStdString();
	}
	else if (root_path.compare("points") == 0)
	{
		QModelIndex frame_index = index(frame-start, 0, root_index);
		folder = filePath(frame_index).toStdString();
	}

	return folder;
}

std::string FileSystemModel::getPointsFilename(int frame)
{
	std::string folder = getPointsFolder(frame);
	if (folder.empty())
		return folder;

	return folder+"/points.pcd";
}

void FileSystemModel::showPointCloud(int frame)
{
	showPointCloud(getPointsFilename(frame));
}

void FileSystemModel::showPointCloud(const std::string& filename)
{
	QModelIndex index = this->index(QString(filename.c_str()));
	if (!index.isValid())
		return;

	showPointCloud(index);

	return;
}

void FileSystemModel::showPointCloud(const QPersistentModelIndex& index)
{
	checked_indexes_.insert(index);

	osg::ref_ptr<PointCloud> point_cloud(getPointCloud(filePath(index).toStdString()));
	if (!point_cloud.valid())
		return;

	PointCloudMap::iterator point_cloud_map_it = point_cloud_map_.find(index);
	if (point_cloud_map_it != point_cloud_map_.end())
		return;

	//MainWindow::getInstance()->getOSGViewerWidget()->addChild(point_cloud);
	point_cloud_map_[index] = point_cloud;

	return;
}

void FileSystemModel::hidePointCloud(int frame)
{
	hidePointCloud(getPointsFilename(frame));
}

void FileSystemModel::hidePointCloud(const std::string& filename)
{
	QModelIndex index = this->index(QString(filename.c_str()));
	if (!index.isValid())
		return;

	hidePointCloud(index);
}


void FileSystemModel::hidePointCloud(const QPersistentModelIndex& index)
{
	checked_indexes_.remove(index);

	PointCloudMap::iterator point_cloud_map_it = point_cloud_map_.find(index);
	if (point_cloud_map_it == point_cloud_map_.end())
		return;

	//MainWindow::getInstance()->getOSGViewerWidget()->removeChild(point_cloud_map_it.value().get());
	point_cloud_map_.erase(point_cloud_map_it);

	return;
}

//void FileSystemModel::hideAndShowPointCloud(int hide_frame, int show_frame)
//{
//	bool to_hide = true;
//	bool to_show = true;
//
//	osg::ref_ptr<PointCloud> show_cloud = getPointCloud(show_frame);
//	osg::ref_ptr<PointCloud> hide_cloud = getPointCloud(hide_frame);
//
//	/*QModelIndex show_index = this->index(QString(getPointsFilename(show_frame).c_str()));
//	checked_indexes_.insert(show_index);
//	PointCloudMap::iterator point_cloud_map_it = point_cloud_map_.find(show_index);
//	if (point_cloud_map_it == point_cloud_map_.end())
//	{
//	if (show_cloud != NULL)
//		point_cloud_map_[show_index] = show_cloud;
//	else
//		to_show = false;
//	}
//	else
//	to_show = false;*/
//
//	/*QModelIndex hide_index = this->index(QString(getPointsFilename(hide_frame).c_str()));
//	checked_indexes_.remove(hide_index);
//	point_cloud_map_it = point_cloud_map_.find(hide_index);
//	if (point_cloud_map_it != point_cloud_map_.end())
//	point_cloud_map_.erase(point_cloud_map_it);
//	else
//	to_hide = false;*/
//
//	/*OSGViewerWidget* osg_viewer_widget = MainWindow::getInstance()->getOSGViewerWidget();
//	if (to_hide && to_show)
//	osg_viewer_widget->replaceChild(hide_cloud, show_cloud, true);
//	else if (to_hide)
//	osg_viewer_widget->removeChild(getPointCloud(hide_frame, hide_view), true);
//	else if (to_show)
//	osg_viewer_widget->addChild(getPointCloud(show_frame, show_view), true);
//
//	showPointCloudSceneInformation();*/
//
//	return;
//}

void FileSystemModel::updatePointCloud(int frame)
{
	std::string filename = getPointsFilename(frame);
	if (point_cloud_cache_map_.find(filename) == point_cloud_cache_map_.end())
		return;

	getPointCloud(frame)->reload();

	return;
}


//PointCloud* FileSystemModel::getDisplayFirstFrame(void)
//{
//	if (point_cloud_map_.empty())
//		return NULL;
//
//	osg::ref_ptr<PointCloud> first_cloud = *point_cloud_map_.begin();
//	int frame = first_cloud->getFrame();
//	int view = first_cloud->getView();
//
//	for (PointCloudMap::const_iterator it = point_cloud_map_.begin(); it != point_cloud_map_.end(); ++ it)
//	{
//	osg::ref_ptr<PointCloud> point_cloud = *it;
//	int this_frame = point_cloud->getFrame();
//	int this_view = point_cloud->getView();
//	if (this_frame < frame)
//	{
//		frame = this_frame;
//		view = this_view;
//	}
//	else if(this_frame == frame && this_view > view)
//	{
//		frame = this_frame;
//		view = this_view;
//	}
//	}
//
//	if (view != 12)
//	return NULL;
//
//	return getPointCloud(frame);
//}
//
//void FileSystemModel::getDisplayFirstFrameFirstView(int& frame, int& view)
//{
//	if (point_cloud_map_.empty())
//	{
//	frame = -1;
//	view = -1;
//	return;
//	}
//
//	osg::ref_ptr<PointCloud> first_cloud = *point_cloud_map_.begin();
//	frame = first_cloud->getFrame();
//	view = first_cloud->getView();
//
//	for (PointCloudMap::const_iterator it = point_cloud_map_.begin(); it != point_cloud_map_.end(); ++ it)
//	{
//	osg::ref_ptr<PointCloud> point_cloud = *it;
//	int this_frame = point_cloud->getFrame();
//	int this_view = point_cloud->getView();
//	if (this_frame < frame)
//	{
//		frame = this_frame;
//		view = this_view;
//	}
//	else if(this_frame == frame && this_view < view)
//	{
//		frame = this_frame;
//		view = this_view;
//	}
//	}
//	return;
//}
//
//void FileSystemModel::getDisplayFirstFrameLastView(int& frame, int& view)
//{
//	if (point_cloud_map_.empty())
//	{
//	frame = -1;
//	view = -1;
//	return;
//	}
//
//	std::vector<std::pair<int, int> > display_items;
//	for (PointCloudMap::const_iterator it = point_cloud_map_.begin(); it != point_cloud_map_.end(); ++ it)
//	{
//	osg::ref_ptr<PointCloud> point_cloud = *it;
//	display_items.push_back(std::make_pair(point_cloud->getFrame(), point_cloud->getView()));
//	}
//
//	frame = display_items[0].first;
//	view = display_items[0].second;
//	for (size_t i = 1, i_end = display_items.size(); i < i_end; ++ i)
//	{
//	int this_frame = display_items[i].first;
//	if (this_frame < frame)
//		frame = this_frame;
//	}
//	for (size_t i = 1, i_end = display_items.size(); i < i_end; ++ i)
//	{
//	int this_frame = display_items[i].first;
//	int this_view = display_items[i].second;
//	if (this_frame == frame && this_view > view)
//		view = this_view;
//	}
//	return;
//}
//
//void FileSystemModel::getDisplayLastFrameLastView(int& frame, int& view)
//{
//	if (point_cloud_map_.empty())
//	{
//	frame = -1;
//	view = -1;
//	return;
//	}
//
//	osg::ref_ptr<PointCloud> first_cloud = *point_cloud_map_.begin();
//	frame = first_cloud->getFrame();
//	view = first_cloud->getView();
//
//	for (PointCloudMap::const_iterator it = point_cloud_map_.begin(); it != point_cloud_map_.end(); ++ it)
//	{
//	osg::ref_ptr<PointCloud> point_cloud = *it;
//	int this_frame = point_cloud->getFrame();
//	int this_view = point_cloud->getView();
//	if (this_frame > frame)
//	{
//		frame = this_frame;
//		view = this_view;
//	}
//	else if(this_frame == frame && this_view > view)
//	{
//		frame = this_frame;
//		view = this_view;
//	}
//	}
//	return;
//}
//
//
//
//void FileSystemModel::navigateToPreviousFrame(NavigationType type)
//{
//	int first_frame, first_view;
//	getDisplayLastFrameLastView(first_frame, first_view);
//
//	if (first_frame == -1 || first_view == -1)
//	{
//	showPointCloud(getStartFrame(), 12);
//	return;
//	}
//
//	if (type == ERASE)
//	{
//	hidePointCloud(first_frame, first_view);
//	return;
//	}
//
//	int current_frame = first_frame-1;
//	if (current_frame < getStartFrame())
//	return;
//
//	if (type == APPEND)
//	showPointCloud(current_frame, first_view);
//	else
//	hideAndShowPointCloud(first_frame, first_view, current_frame, first_view);
//
//	return;
//}
//
//void FileSystemModel::navigateToNextFrame(NavigationType type)
//{
//	int last_frame, last_view;
//	getDisplayFirstFrameLastView(last_frame, last_view);
//
//	if (last_frame == -1 || last_view == -1)
//	{
//	showPointCloud(getEndFrame(), 12);
//	return;
//	}
//
//	if (type == ERASE)
//	{
//	hidePointCloud(last_frame, last_view);
//	return;
//	}
//
//	int current_frame = last_frame+1;
//	if (current_frame > getEndFrame())
//	return;
//
//	if (type == APPEND)
//	showPointCloud(current_frame, last_view);
//	else
//	hideAndShowPointCloud(last_frame, last_view, current_frame, last_view);
//
//	return;
//}
//
//void FileSystemModel::navigateToPreviousView(NavigationType type)
//{
//	int first_frame, first_view;
//	getDisplayFirstFrameFirstView(first_frame, first_view);
//
//	if (first_frame == -1 || first_view == -1)
//	{
//	showPointCloud(getStartFrame(), 0);
//	return;
//	}
//
//	if (type == ERASE)
//	{
//	hidePointCloud(first_frame, first_view);
//	return;
//	}
//
//	int current_view = first_view-1;
//	if (current_view < 0)
//	return;
//
//	if (type == APPEND)
//	showPointCloud(first_frame, current_view);
//	else
//	hideAndShowPointCloud(first_frame, first_view, first_frame, current_view);
//
//	return;
//}
//
//void FileSystemModel::navigateToNextView(NavigationType type)
//{
//	int first_frame, last_view;
//	getDisplayFirstFrameLastView(first_frame, last_view);
//
//	if (first_frame == -1 || last_view == -1)
//	{
//	showPointCloud(getStartFrame(), 11);
//	return;
//	}
//
//	if (type == ERASE)
//	{
//	hidePointCloud(first_frame, last_view);
//	return;
//	}
//
//	int current_view = last_view+1;
//	if (current_view > 12)
//	return;
//
//	if (type == APPEND)
//	showPointCloud(first_frame, current_view);
//	else
//	hideAndShowPointCloud(first_frame, last_view, first_frame, current_view);
//
//	return;
//}
