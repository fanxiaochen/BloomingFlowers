
#include <QDir>
#include <QMutexLocker>

#include <osg/Group>
#include <osgDB/ReadFile>

#include <pcl/io/ply_io.h>


#include "main_window.h"
#include "point_cloud.h"
#include "osg_viewer_widget.h"
#include "points_file_system.h"
#include "scene_widget.h"
#include "parameters.h"
#include "task_thread.h"
#include "flower.h"

PointsFileSystem::PointsFileSystem()
    :start_frame_(-1),
    end_frame_(-1)
{
    setNameFilterDisables(false);
    QStringList allowed_file_extensions;
    allowed_file_extensions.push_back("*.pcd");
    setNameFilters(allowed_file_extensions);

    /*connect(this, SIGNAL(timeToHideAndShowPointCloud(int, int, int, int)), this, SLOT(hideAndShowPointCloud(int, int, int, int)));
    connect(this, SIGNAL(timeToShowPointCloud(int, int)), this, SLOT(showPointCloud(int, int)));*/
}

PointsFileSystem::~PointsFileSystem()
{
}

QModelIndex PointsFileSystem::setRootPath(const QString & root_path)
{
    point_cloud_cache_map_.clear();
    point_cloud_map_.clear();
    checked_indexes_.clear();

    QModelIndex index = FileSystemModel::setRootPath(root_path);
    computeFrameRange();

    if (start_frame_ != -1)
    {
        if (getPointCloud(start_frame_) != NULL)
            showPointCloud(start_frame_);
    }

    MainWindow::getInstance()->getSceneWidget()->centerScene();

    return index;
}

bool PointsFileSystem::setData(const QModelIndex &index, const QVariant &value, int role)
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

    return FileSystemModel::setData(index, value, role);
}

bool PointsFileSystem::isShown(const std::string& filename) const
{
    QModelIndex index = this->index(filename.c_str());
    return (checked_indexes_.contains(index)) ? (true) : (false);
}



//void PointsFileSystem::limitPointCloudCacheSize(void)
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

void PointsFileSystem::computeFrameRange(void)
{
    start_frame_ = end_frame_ = -1;

    QString root_path = rootPath();
    QModelIndex root_index = index(root_path);

    if (root_path.contains("frame_")) {
        start_frame_ = end_frame_ = root_path.right(4).toInt();
        return;
    }

    if (root_path.contains("points"))
    {
        QStringList points_entries = QDir(root_path).entryList();
        extractStartEndFrame(points_entries, start_frame_, end_frame_);
        return;
    }

    return;
}

void PointsFileSystem::getFrameRange(int &start, int &end)
{
    start = start_frame_;
    end = end_frame_;
}

osg::ref_ptr<PointCloud> PointsFileSystem::getPointCloud(const std::string& filename)
{
    QMutexLocker locker(&mutex_);

    //limitPointCloudCacheSize();

    QFileInfo fileinfo(filename.c_str());
    if (!fileinfo.exists() || !fileinfo.isFile())
        return NULL;

    PointCloudCacheMap::iterator it = point_cloud_cache_map_.find(filename);
    if (it != point_cloud_cache_map_.end())
        return it->second.get();

    osg::ref_ptr<PointCloud> point_cloud = new PointCloud;
    if (!point_cloud->open(filename))
        return NULL;

    if (point_cloud->thread() != qApp->thread())
        point_cloud->moveToThread(qApp->thread());

    point_cloud_cache_map_[filename] = point_cloud;

    return point_cloud;
}

osg::ref_ptr<PointCloud> PointsFileSystem::getPointCloud(int frame)
{
    return getPointCloud(getPointsFilename(frame));
}


std::string PointsFileSystem::getPointsFolder(int frame)
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
    else if (root_path.contains("points"))
    {
        // not work ??
        /*std::cout << root_path.toStdString() << std::endl;
        QModelIndex frame_index = index(frame-start, 0, root_index);
        folder = filePath(frame_index).toStdString();*/

        QString frame_name(QString("frame_%1").arg(frame, 5, 10, QChar('0')));
        QStringList root_entries = QDir(root_path).entryList();
        for (QStringList::const_iterator root_entries_it = root_entries.begin();
            root_entries_it != root_entries.end(); ++ root_entries_it)
        {
            if (root_entries_it->compare(frame_name) == 0)
            {
                folder = (root_path + QString("/%1").arg(*root_entries_it)).toStdString();
                break;
            }
        }
    }

    return folder;
}

std::string PointsFileSystem::getPointsFilename(int frame)
{
    std::string folder = getPointsFolder(frame);
    if (folder.empty())
        return folder;

    return folder+"/points.pcd";
}

std::string PointsFileSystem::getClosureCloudFilename(int frame)
{
    std::string folder = getPointsFolder(frame);
    if (folder.empty())
        return folder;

    return folder+"/points-closure.obj";
}

void PointsFileSystem::savePointCloudAsPly(int frame)
{
    std::string folder = getPointsFolder(frame);
    if (folder.empty())
        return;

    PointCloud* point_cloud = getPointCloud(frame);
    point_cloud->reEstimateNormal();

    std::string save_ply = folder + "/points.ply";
    pcl::io::savePLYFileASCII (save_ply, *point_cloud); 
}

void PointsFileSystem::showPointCloud(int frame)
{
    showPointCloud(getPointsFilename(frame));
}

void PointsFileSystem::showPointCloud(const std::string& filename)
{
    QModelIndex index = this->index(QString(filename.c_str()));
    if (!index.isValid())
        return;

    showPointCloud(index);

    return;
}

osg::ref_ptr<PointCloud> PointsFileSystem::getPointCloud(QPersistentModelIndex index)
{
    return point_cloud_map_[index];
}

void PointsFileSystem::showPointCloud(const QPersistentModelIndex& index)
{
    checked_indexes_.insert(index);

    osg::ref_ptr<PointCloud> point_cloud(getPointCloud(filePath(index).toStdString()));
    if (!point_cloud.valid())
        return;

    PointCloudMap::iterator point_cloud_map_it = point_cloud_map_.find(index);
    if (point_cloud_map_it != point_cloud_map_.end())
        return;

    MainWindow::getInstance()->getSceneWidget()->addSceneChild(point_cloud);
    point_cloud_map_[index] = point_cloud;

    return;
}

void PointsFileSystem::hidePointCloud(int frame)
{
    hidePointCloud(getPointsFilename(frame));
}

void PointsFileSystem::hidePointCloud(const std::string& filename)
{
    QModelIndex index = this->index(QString(filename.c_str()));
    if (!index.isValid())
        return;

    hidePointCloud(index);
}


void PointsFileSystem::hidePointCloud(const QPersistentModelIndex& index)
{
    checked_indexes_.remove(index);

    PointCloudMap::iterator point_cloud_map_it = point_cloud_map_.find(index);
    if (point_cloud_map_it == point_cloud_map_.end())
        return;

    MainWindow::getInstance()->getSceneWidget()->removeSceneChild(point_cloud_map_it.value().get());
    point_cloud_map_.erase(point_cloud_map_it);

    return;
}

void PointsFileSystem::hideAndShowPointCloud(int hide_frame, int show_frame)
{
    bool to_hide = true;
    bool to_show = true;

    osg::ref_ptr<PointCloud> show_cloud = getPointCloud(show_frame);
    osg::ref_ptr<PointCloud> hide_cloud = getPointCloud(hide_frame);

    QModelIndex show_index = this->index(QString(getPointsFilename(show_frame).c_str()));
    checked_indexes_.insert(show_index);
    PointCloudMap::iterator point_cloud_map_it = point_cloud_map_.find(show_index);
    if (point_cloud_map_it == point_cloud_map_.end())
    {
        if (show_cloud != NULL)
            point_cloud_map_[show_index] = show_cloud;
        else
            to_show = false;
    }
    else
        to_show = false;

    QModelIndex hide_index = this->index(QString(getPointsFilename(hide_frame).c_str()));
    checked_indexes_.remove(hide_index);
    point_cloud_map_it = point_cloud_map_.find(hide_index);
    if (point_cloud_map_it != point_cloud_map_.end())
        point_cloud_map_.erase(point_cloud_map_it);
    else
        to_hide = false;

    SceneWidget* scene_widget = MainWindow::getInstance()->getSceneWidget();
    if (to_hide && to_show)
        scene_widget->replaceSceneChild(hide_cloud, show_cloud);
    else if (to_hide)
    scene_widget->removeSceneChild(getPointCloud(hide_frame));
    else if (to_show)
    scene_widget->addSceneChild(getPointCloud(show_frame));

    return;
}

void PointsFileSystem::updatePointCloud(int frame)
{
    std::string filename = getPointsFilename(frame);
    if (point_cloud_cache_map_.find(filename) == point_cloud_cache_map_.end())
        return;

    getPointCloud(frame)->reload();

    return;
}


PointCloud* PointsFileSystem::getDisplayFirstFrame(void)
{
    if (point_cloud_map_.empty())
        return NULL;

    osg::ref_ptr<PointCloud> first_cloud = *point_cloud_map_.begin();
    int frame = first_cloud->getFrame();

    for (PointCloudMap::const_iterator it = point_cloud_map_.begin(); it != point_cloud_map_.end(); ++ it)
    {
        osg::ref_ptr<PointCloud> point_cloud = *it;
        int this_frame = point_cloud->getFrame();
   
        if (this_frame < frame)
        {
            frame = this_frame;
        }
    }

    return getPointCloud(frame);
}


void PointsFileSystem::navigateToPreviousFrame()
{
    int current_frame = getDisplayFirstFrame()->getFrame();

    if (current_frame == getStartFrame())
        return;

    int prev_frame = current_frame - 1;

    hideAndShowPointCloud(current_frame, prev_frame);

    return;
}

void PointsFileSystem::navigateToNextFrame()
{
    int current_frame = getDisplayFirstFrame()->getFrame();

    if (current_frame == getEndFrame())
        return;

    int next_frame = current_frame + 1;

    hideAndShowPointCloud(current_frame, next_frame);

    return;
}