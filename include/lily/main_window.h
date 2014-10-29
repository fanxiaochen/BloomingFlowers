
#ifndef MainWindow_H
#define MainWindow_H

#include <string>
#include <cassert>

#include <QMutex>
#include <QMainWindow>

#include "ui_main_window.h"

class SceneWidget;
class FileSystemModel;
class FileViewerWidget;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
	MainWindow();
	void init(void);
	virtual ~MainWindow();
	static MainWindow* getInstance();

	inline SceneWidget* getSceneWidget() { return scene_widget_; }
	inline FileSystemModel* getFileSystemModel() { return file_system_model_; }
	inline FileViewerWidget* getFileViewerWidget() { return file_viewer_widget_;}

	inline std::string& getWorkspace(){ return workspace_; }

	void showInformation(const std::string& information);
	void showStatus(const std::string& status, int timeout=0);

public slots:
	bool slotShowYesNoMessageBox(const std::string& text, const std::string& informative_text);

signals:
	void keyDownPressed(void);
	void showInformationRequested(const QString& information);
	void showStatusRequested(const QString& status, int timeout);

protected:
	virtual void closeEvent(QCloseEvent *event);
	virtual void keyPressEvent(QKeyEvent* event);

private slots:
	void slotShowInformation(const QString& information);
	void slotShowStatus(const QString& status, int timeout);
	bool slotSetWorkspace();

private:
	void loadSettings();
	void saveSettings();
	void saveStatusLog();

private:
	Ui::MainWindowClass             ui_;

	std::string						workspace_;
	SceneWidget*					scene_widget_;
	FileViewerWidget*				file_viewer_widget_;
	FileSystemModel*				file_system_model_;
};

class MainWindowInstancer
{
public:
  static MainWindowInstancer& getInstance() {
    static MainWindowInstancer singleton_;
    return singleton_;
  }

private:
  MainWindowInstancer():main_window_(NULL){}
  MainWindowInstancer(const MainWindowInstancer &) {}            // copy ctor hidden
  MainWindowInstancer& operator=(const MainWindowInstancer &) {return (*this);}   // assign op. hidden
  virtual ~MainWindowInstancer(){}

  friend class MainWindow;
  MainWindow*   main_window_;
};

#endif // MainWindow_H
