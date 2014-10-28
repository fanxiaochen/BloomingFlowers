#include <QMenu>
#include <QFileSystemModel>

#include "main_window.h"
#include "point_cloud.h"

#include "file_system_model.h"
#include "file_viewer_widget.h"

FileViewerWidget::FileViewerWidget(QWidget * parent)
  : QTreeView(parent),
  model_(new FileSystemModel)
{
  setModel(model_);

  setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

  hideColumn(1);
  hideColumn(2);
  hideColumn(3);
}

FileViewerWidget::~FileViewerWidget(void)
{

}

void FileViewerWidget::setWorkspace(const std::string& workspace)
{
  QModelIndex root_index = model_->setRootPath(workspace.c_str());
  setRootIndex(root_index);

  return;
}
