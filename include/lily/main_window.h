
#ifndef MainWindow_H
#define MainWindow_H

#include <string>
#include <cassert>

#include <QMutex>
#include <QMainWindow>

#include "ui_main_window.h"

class FileSystemModel;
class FileViewerWidget;
class SceneWidget;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
	MainWindow();
	void init(void);
	virtual ~MainWindow();
	static MainWindow* getInstance();

	inline SceneWidget* getSceneWidget() { return scene_widget_; }
	inline FileSystemModel* getPointsSystem() { return points_files_; }
	inline FileSystemModel* getMeshSystem() { return mesh_files_; }
	inline FileViewerWidget* getPointsWidget() { return points_widget_; }
	inline FileViewerWidget* getMeshWidget() { return mesh_widget_; }

	inline std::string& getPointsPath(){ return points_path_; }
	inline std::string& getMeshPath(){ return mesh_path_; }

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
	bool slotLoadPoints();
	bool slotLoadMesh();

private:
	void loadSettings();
	void saveSettings();
	void saveStatusLog();

private:
	Ui::MainWindowClass             ui_;

	std::string						points_path_;
	std::string						mesh_path_;
	SceneWidget*					scene_widget_;
	FileViewerWidget*				points_widget_;
	FileViewerWidget*				mesh_widget_;
	FileSystemModel*				points_files_;
	FileSystemModel*				mesh_files_;
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
