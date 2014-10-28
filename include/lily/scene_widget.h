
#ifndef SCENE_WIDGET_H_
#define SCENE_WIDGET_H_

#include "renderable.h"

#include "osg_viewer_widget.h"

class SceneWidget : public OSGViewerWidget
{
  Q_OBJECT

public:
	SceneWidget(QWidget * parent = 0, const QGLWidget * shareWidget = 0, Qt::WindowFlags f = 0);
	virtual ~SceneWidget();

	virtual QSize sizeHint() const {return QSize(256, 256);}

	Renderable* getRenderable(void);

	void setRenderable(Renderable* renderable);

public slots:
	/*void slotOpenPointCloud(void);
	void slotSavePointCloud(void);

	void slotSensorFromCamera(void);
	void slotCameraFromSensor(void);

	void slotEstimateNormals(void);
	void slotEstimateCurvatures(void);
	void slotOrientNormals(void);
	void slotFlipAllNormals(void);
	void slotVoxelGridFilter(void);

	void slotColorizeOriginal(void);
	void slotColorizeUniform(void);
	void slotColorizeCategory(void);
	void slotColorizeInstance(void);
	void slotColorizeSegment(void);
	void slotColorizeCurvature(void);
	void slotColorizeDepth(void);
	void slotColorizeIntensity(void);

	void slotTogglePointCloud(void);

	void slotToggleShowNormals(void);
	void slotToggleMeshModels(void);

	void slotToggleCloudDraggers(void);
	void slotToggleCloudScalers(void);

	void slotPOVRaySnapshot(void);
	void slotPOVRayVideo(void);

	void slotImportMeshModels(void);
	void slotSampleMeshModels(void);
	void slotClearMeshModels(void);
	void slotVirtualScan(void);*/

private:
	osg::ref_ptr<Renderable>              renderable_;
};

#endif // SCENE_WIDGET_H_
