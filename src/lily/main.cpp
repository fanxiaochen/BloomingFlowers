#include <QApplication>

#include "file_viewer_widget.h"
#include "file_system_model.h"

#include "main_window.h"

int main(int argc, char *argv[])
{
  // Make Xlib and GLX thread safe under X11
  QApplication::setAttribute(Qt::AA_X11InitThreads);
  QApplication application(argc, argv);

  MainWindow main_window;
  main_window.showMaximized();

 // QFileSystemModel mm;
 // FileViewerWidget w(NULL, &mm);
 //// QTreeView ww;
 // 
 //   
 //   //m.setFilter(QDir::Dirs | QDir::NoDotAndDotDot);
 //   QModelIndex index = mm.setRootPath("C:\\");
 //
 //   w.setModel(&mm);
 //   w.setRootIndex(index);
 //   w.hideColumn(3);
 //   w.hideColumn(2);
 //   w.hideColumn(1);
 //
 //   w.show();

  return application.exec();
}