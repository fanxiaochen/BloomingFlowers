
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
// #include "vertex_pickup.h"


SceneWidget::SceneWidget(QWidget * parent, const QGLWidget * shareWidget, Qt::WindowFlags f)
	:OSGViewerWidget(parent, shareWidget, f), light_number_(6), light_status_(false)
{
    osg::ref_ptr<osg::LightModel> light_model = new osg::LightModel();
    light_model->setTwoSided(true);
	light_model->setAmbientIntensity( osg::Vec4(0.6,0.6,0.6,0.5));
    scene_root_->getOrCreateStateSet()->setAttributeAndModes(light_model.get(), osg::StateAttribute::ON);

	scene_root_->getOrCreateStateSet()->setMode( GL_LIGHTING, osg::StateAttribute::ON );
	scene_root_->getOrCreateStateSet()->setMode( GL_LIGHT0, osg::StateAttribute::ON );
	scene_root_->getOrCreateStateSet()->setMode( GL_LIGHT1, osg::StateAttribute::ON );
	scene_root_->getOrCreateStateSet()->setMode( GL_LIGHT2, osg::StateAttribute::ON );
	scene_root_->getOrCreateStateSet()->setMode( GL_LIGHT3, osg::StateAttribute::ON );
	scene_root_->getOrCreateStateSet()->setMode( GL_LIGHT4, osg::StateAttribute::ON );
	scene_root_->getOrCreateStateSet()->setMode( GL_LIGHT5, osg::StateAttribute::ON );

    addEventHandler(new PickHandler());
    addEventHandler(new CameraHandler());
    addEventHandler(new NodeHandler(scene_root_));
	addEventHandler(new osgViewer::ScreenCaptureHandler(new WriteToFile(QString(""))));	
//	addEventHandler(new PickupVertexHandler(other_root_.get()));
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
    double scale = 20.0;
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
        light->setAmbient( osg::Vec4( .1f, .1f, .1f, 1.f ));
        light->setDiffuse( osg::Vec4( .8f, .8f, .8f, 1.f ));
        light->setSpecular( osg::Vec4( .8f, .8f, .8f, 1.f ));
        //light->setDirection( osg::Vec3( 1.f, 0.f, 0.f ));
        //light->setSpotCutoff(25.f );
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
