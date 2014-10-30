#include <QMenu>
#include <QFile>
#include <QFileDialog>
#include <QMetaObject>
#include <QElapsedTimer>

#include <osg/LightModel>

#include "main_window.h"

#include "scene_widget.h"

SceneWidget::SceneWidget(QWidget * parent, const QGLWidget * shareWidget, Qt::WindowFlags f)
	:OSGViewerWidget(parent, shareWidget, f)
{
	osg::ref_ptr<osg::LightModel> light_model = new osg::LightModel();
	light_model->setTwoSided(true);
	scene_root_->getOrCreateStateSet()->setAttributeAndModes(light_model.get(), osg::StateAttribute::ON);

	//addSceneChild(point_cloud_);
}

SceneWidget::~SceneWidget()
{

}
