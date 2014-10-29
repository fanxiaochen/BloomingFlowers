
#include "file_system_model.h"

FileSystemModel::FileSystemModel()
{
	/*setNameFilterDisables(false);
	QStringList file_extensions;
	file_extensions.push_back("*.pcd");
	file_extensions.push_back("*.ply");
	setNameFilters(file_extensions);*/
}

FileSystemModel::~FileSystemModel()
{

}

//QModelIndex FileSystemModel::setRootPath(const QString & new_path)
//{
//	root_path_ = new_path;
//	QModelIndex index = QFileSystemModel::setRootPath(new_path);
//	return index;
//}
//
//Qt::ItemFlags FileSystemModel::flags(const QModelIndex &index) const
//{
//  return QFileSystemModel::flags(index) | Qt::ItemIsUserCheckable;
//}
//
//QVariant FileSystemModel::data(const QModelIndex &index, int role) const
//{
//  /*if(role == Qt::CheckStateRole)
//    return computeCheckState(index);
//  else
//  {
//    if(role == Qt::ForegroundRole && checkRegisterState(index))
//      return QBrush(QColor(255, 0, 0));
//    return QFileSystemModel::data(index, role);
//  }*/
//	QVariant val;
//	return val;
//
//}

