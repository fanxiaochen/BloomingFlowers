#ifndef FILE_VIEWER_WIDGET_H
#define FILE_VIEWER_WIDGET_H

#include <QTreeView>

class FileSystemModel;

class FileViewerWidget : public QTreeView
{
public:
	FileViewerWidget(QWidget * parent, FileSystemModel* file_system_model);
	virtual ~FileViewerWidget(void);

	virtual QSize sizeHint() const {return QSize(320, 480);}
	void setWorkspace(QString& workspace);
	inline FileSystemModel* getFileSystemModel() { return file_system_model_; }

private:
	FileSystemModel*  file_system_model_;
};

#endif /*FILE_VIEWER_WIDGET_H*/