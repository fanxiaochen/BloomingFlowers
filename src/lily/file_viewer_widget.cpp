
#include "file_system_model.h"
#include "file_viewer_widget.h"

FileViewerWidget::FileViewerWidget(QWidget * parent, FileSystemModel* file_system_model)
  : QTreeView(parent),
  file_system_model_(file_system_model)
{

	setModel(file_system_model_);

	setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

	hideColumn(1);
	hideColumn(2);
	hideColumn(3);
}

FileViewerWidget::~FileViewerWidget(void)
{

}

void FileViewerWidget::setWorkspace(QString& workspace)
{
	QModelIndex root_index = file_system_model_->setRootPath(workspace);
	setRootIndex(root_index);

	return;
}
