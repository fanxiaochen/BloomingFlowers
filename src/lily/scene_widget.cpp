
#include <QMenu>
#include <QFile>
#include <QFileDialog>
#include <QMetaObject>
#include <QElapsedTimer>

#include <osg/LightModel>
#include <osg/ShapeDrawable>

#include "main_window.h"
#include "pick_handler.h"
#include "scene_widget.h"


SceneWidget::SceneWidget(QWidget * parent, const QGLWidget * shareWidget, Qt::WindowFlags f)
	:OSGViewerWidget(parent, shareWidget, f), light_number_(6), light_status_(false)
{
	osg::ref_ptr<osg::LightModel> light_model = new osg::LightModel();
	light_model->setTwoSided(true);
	scene_root_->getOrCreateStateSet()->setAttributeAndModes(light_model.get(), osg::StateAttribute::ON);

    addEventHandler(new PickHandler());
    addEventHandler(new CameraHandler());
    addEventHandler(new NodeHandler(scene_root_));
	addEventHandler(new osgViewer::ScreenCaptureHandler(new WriteToFile(QString(""))));	
}

SceneWidget::~SceneWidget()
{

}

bool SceneWidget::setLight()
{
    light_status_ = !light_status_;

    if (light_status_)
        turnOnLights();
    else
        turnOffLights();

    return true;
}

void SceneWidget::turnOnLights()
{
    osg::BoundingSphere bounding_sphere = scene_root_->getBound();
    osg::Vec3 center = bounding_sphere.center();
    double radius = bounding_sphere.radius();
    double scale = 2.0;
    std::vector<osg::Vec4> light_pos;
    light_pos.push_back(osg::Vec4(center.x()+scale*radius, center.y(), center.z(), 1));
    light_pos.push_back(osg::Vec4(center.x()-scale*radius, center.y(), center.z(), 1));
    light_pos.push_back(osg::Vec4(center.x(), center.y()+scale*radius, center.z(), 1));
    light_pos.push_back(osg::Vec4(center.x(), center.y()-scale*radius, center.z(), 1));
    light_pos.push_back(osg::Vec4(center.x(), center.y(), center.z()+scale*radius, 1));
    light_pos.push_back(osg::Vec4(center.x(), center.y(), center.z()-scale*radius, 1));

    for (size_t i = 0; i < light_number_; ++ i)
    {
        osg::Light* light = new osg::Light;
        light->setLightNum(i);
        light->setDiffuse(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
        light->setAmbient(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
        light->setSpecular(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
        light->setPosition(light_pos[i]);

        osg::ref_ptr<osg::LightSource> ls = new osg::LightSource;
        ls->setLight(light);
        addSceneChild(ls);
        light_sources_.push_back(ls);
    }
}

void SceneWidget::turnOffLights()
{
    for (size_t i = 0; i < light_number_; ++ i)
    {
        osg::ref_ptr<osg::LightSource> ls = light_sources_.at(i);
        removeSceneChild(ls);
    }

    light_sources_.clear();
}
