
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
class TrackingSystem;
class Parameters;
class Registrator;
class FlowersViewer;
class QCheckBox;
class TrajectoryModel;

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
    inline TrackingSystem* getTrackingSystem() { return tracking_system_; }
    inline Parameters* getParameters(){ return parameters_; }
    inline Registrator* getRegistrator(){ return registrator_; }
    inline FlowersViewer* getFlowersViewer(){ return flowers_viewer_; }

    inline std::string& getPointsPath(){ return points_path_; }
    inline std::string& getMeshPath(){ return mesh_path_; }
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
    bool slotLoadPoints();
    bool slotLoadMesh();
    bool slotLoadParameters();
    bool slotSaveParameters();
	bool loadCamera();
	bool saveCamera();
    bool slotLoadAxis();
    bool slotLoadFlowers();
    bool slotSendCheckBoxRenderState();
	bool slotSnapshotAllFrames();

    bool region_probability();
	bool collision_detection();
    bool trajectories_generation();
    bool merge_petals();
    bool petal_sequences();
	bool save_plys();
    bool camera_views();
    bool sequence_smoothing();

    bool transfer();
    bool multi_layer();
    bool interpolation();

private:
    void loadSettings();
    void saveSettings();
    void saveStatusLog();

private:
    void setRenderingBox();

private:
    Ui::MainWindowClass             ui_;

    std::string                     workspace_;
    std::string						points_path_;
    std::string						mesh_path_;
    SceneWidget*					scene_widget_;
    FileViewerWidget*				points_widget_;
    FileViewerWidget*				mesh_widget_;
    FileSystemModel*				points_files_;
    FileSystemModel*				mesh_files_;

    TrackingSystem*					tracking_system_;
    Parameters*                     parameters_;
    Registrator*                    registrator_;
    FlowersViewer*                  flowers_viewer_;

    std::vector<QCheckBox*>         Check_list_;

    TrajectoryModel*                trajectory_model_;
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
