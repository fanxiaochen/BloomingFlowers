#pragma once
#ifndef FILE_SYSTEM_MODEL_H
#define FILE_SYSTEM_MODEL_H

#include <unordered_map>

#include <QSet>
#include <QHash>
#include <QMutex>
#include <QFileSystemModel>
#include <QPersistentModelIndex>

#include <osg/ref_ptr>
#include <osg/Vec4>

class PointCloud;

class FileSystemModel : public QFileSystemModel
{
  Q_OBJECT

public:
	FileSystemModel();
	virtual ~FileSystemModel();

	QModelIndex setRootPath ( const QString & root_path );
	Qt::ItemFlags flags(const QModelIndex &index) const;
	QVariant data(const QModelIndex &index, int role) const;
	bool setData(const QModelIndex &index, const QVariant &value, int role);

	bool isShown(const std::string& filename) const;

	void getFrameRange(int &start, int &end);
	int getStartFrame(void) const {return start_frame_;}
	int getEndFrame(void) const {return end_frame_;}

	std::string getPointsFolder(int frame);
	std::string getPointsFilename(int frame);

	osg::ref_ptr<PointCloud> getPointCloud(const std::string& filename);
	osg::ref_ptr<PointCloud> getPointCloud(int frame);

	void showPointCloud(const std::string& filename);
	void hidePointCloud(const std::string& filename);

	void showPointCloud(const QPersistentModelIndex& index);
	void hidePointCloud(const QPersistentModelIndex& index);

	void showPointCloud(int frame);
	void hidePointCloud(int frame);

	void updatePointCloud(int frame);



	/*PointCloud* getDisplayFirstFrame(void);

	enum NavigationType
	{
	SWITCH,
	APPEND,
	ERASE
	};

	void navigateToPreviousFrame(NavigationType type);
	void navigateToNextFrame(NavigationType type);
	void navigateToPreviousView(NavigationType type);
	void navigateToNextView(NavigationType type);*/

	/*void showPointCloud(int frame, int view);
	void hideAndShowPointCloud(int hide_frame, int hide_view, int show_frame, int show_view);*/

private:
	Qt::CheckState computeCheckState(const QModelIndex &index) const;
	bool recursiveCheck(const QModelIndex &index, const QVariant &value);	

	//void limitPointCloudCacheSize(void);
	void computeFrameRange(void);

	/*void getDisplayFirstFrameFirstView(int& frame, int& view);
	void getDisplayFirstFrameLastView(int& frame, int& view);
	void getDisplayLastFrameLastView(int& frame, int& view);*/

	
signals:
	/*void timeToHideAndShowPointCloud(int hide_frame, int hide_view, int show_frame, int show_view);
	void timeToShowPointCloud(int show_frame, int show_view);*/

private:
	typedef QHash<QPersistentModelIndex, osg::ref_ptr<PointCloud> > PointCloudMap;
    typedef std::unordered_map<std::string, osg::ref_ptr<PointCloud> > PointCloudCacheMap;

	PointCloudMap					point_cloud_map_;
	PointCloudCacheMap				point_cloud_cache_map_;

  	QSet<QPersistentModelIndex>     checked_indexes_;
  

	int								start_frame_;
	int								end_frame_;
	QMutex							mutex_;
};
#endif // FILE_SYSTEM_MODEL_H