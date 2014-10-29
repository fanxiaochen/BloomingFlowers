#ifndef FILE_SYSTEM_MODEL_H
#define FILE_SYSTEM_MODEL_H

#include <QFileSystemModel>

class FileSystemModel: public QFileSystemModel
{
public:
	FileSystemModel();
	virtual ~FileSystemModel();

	/*QModelIndex setRootPath (const QString & new_path);
	Qt::ItemFlags flags(const QModelIndex &index) const;
	QVariant data(const QModelIndex &index, int role) const;*/
	//bool setData(const QModelIndex &index, const QVariant &value, int role);
	//bool isShown(const std::string& filename) const;

};
#endif