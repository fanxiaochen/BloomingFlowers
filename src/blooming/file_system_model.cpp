
#include "lily.h"


FileSystemModel::FileSystemModel()
{
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

QModelIndex FileSystemModel::setRootPath(const QString & root_path)
{
	QModelIndex index = QFileSystemModel::setRootPath(root_path);
	return index;
}

bool FileSystemModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
	return QFileSystemModel::setData(index, value, role);
}

