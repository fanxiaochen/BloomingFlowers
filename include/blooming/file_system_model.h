
#ifndef FILE_SYSTEM_MODEL_H
#define FILE_SYSTEM_MODEL_H

#include <QMutex>
#include <QSet>
#include <QFileSystemModel>
#include <QPersistentModelIndex>

class FileSystemModel : public QFileSystemModel
{

public:
	FileSystemModel();
	virtual ~FileSystemModel();

	Qt::ItemFlags flags(const QModelIndex &index) const;
	QVariant data(const QModelIndex &index, int role) const;

	virtual QModelIndex setRootPath ( const QString & root_path );
	virtual bool setData(const QModelIndex &index, const QVariant &value, int role);

protected:
	Qt::CheckState computeCheckState(const QModelIndex &index) const;
	bool recursiveCheck(const QModelIndex &index, const QVariant &value);	

protected:

  	QSet<QPersistentModelIndex>     checked_indexes_;
	QMutex							mutex_;
};
#endif // FILE_SYSTEM_MODEL_H