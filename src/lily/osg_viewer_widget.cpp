
#include <QFileDialog>
#include <QTextStream>
#include <QResizeEvent>

#include <osg/Depth>
#include <osg/Point>
#include <osg/LineWidth>
#include <osg/ComputeBoundsVisitor>
#include <osgGA/TrackballManipulator>
#include <osgGA/StateSetManipulator>

#include "main_window.h"
#include "toggle_handler.h"
#include "registrator.h"
#include "osg_viewer_widget.h"

QMutex OSGViewerWidget::mutex_first_frame_;

OSGViewerWidget::OSGViewerWidget(QWidget * parent, const QGLWidget * shareWidget, Qt::WindowFlags f)
    :AdapterWidget(parent, shareWidget, f, true),
    scene_root_(new osg::Group),
    other_root_(new osg::Group),
    gl_thread_(this),
    threaded_painter_(this),
    first_frame_(true),
    up_vector_(0.0f, -1.0f, 0.0f),
    center_scene_(false)
{
    osg::ref_ptr<osg::Group> root = new osg::Group;
    setSceneData(root);

    root->addChild(other_root_);
    root->addChild(scene_root_);
    scene_root_->getOrCreateStateSet()->setAttribute(new osg::Point(2.0f), osg::StateAttribute::OFF);
    scene_root_->getOrCreateStateSet()->setAttribute(new osg::LineWidth(1.0f), osg::StateAttribute::OFF);

    setCameraManipulator(new osgGA::TrackballManipulator);

    addEventHandler(new osgViewer::HelpHandler);
    //addEventHandler(new osgViewer::StatsHandler);

    double w = width();
    double h = height();
    getCamera()->setViewport(new osg::Viewport(0, 0, w, h));
    getCamera()->setProjectionMatrixAsPerspective(60.0f, w/h, 1.0f, 10000.0f);
    getCamera()->setGraphicsContext(getGraphicsWindow());
    getCamera()->setClearColor(osg::Vec4(1, 1, 1, 1.0));

    setThreadingModel(osgViewer::Viewer::SingleThreaded);

    this->doneCurrent();
}

OSGViewerWidget::~OSGViewerWidget()
{
    osgViewer::View::EventHandlers handlers = getEventHandlers();
    for (osgViewer::View::EventHandlers::iterator it = handlers.begin(); it != handlers.end(); ++ it)
    this->removeEventHandler(*it);

    stopRendering();
}

void OSGViewerWidget::eventTraversal()
{
    double reference_time = _frameStamp->getReferenceTime();
    _frameStamp->setReferenceTime(reference_time*100);

    osgViewer::Viewer::eventTraversal();

    _frameStamp->setReferenceTime(reference_time);

    return;
}

void OSGViewerWidget::increaseLineWidth(void)
{
    osg::StateSet* state_Set = scene_root_->getOrCreateStateSet();
    osg::LineWidth* line_width = dynamic_cast<osg::LineWidth*>(state_Set->getAttribute(osg::StateAttribute::LINEWIDTH));
    if (line_width == NULL)
    return;

    if (line_width->getWidth() >= 16.0)
    {
    line_width->setWidth(16.0);
    return;
    }

    line_width->setWidth(line_width->getWidth()+1.0);

    return;
}

void OSGViewerWidget::decreaseLineWidth(void)
{
    osg::StateSet* state_Set = scene_root_->getOrCreateStateSet();
    osg::LineWidth* line_width = dynamic_cast<osg::LineWidth*>(state_Set->getAttribute(osg::StateAttribute::LINEWIDTH));
    if (line_width == NULL)
    return;

    if (line_width->getWidth() <= 1.0)
    {
    line_width->setWidth(1.0);
    return;
    }

    line_width->setWidth(line_width->getWidth()-1.0);

    return;
}

void OSGViewerWidget::increasePointSize(void)
{
    osg::StateSet* state_set = scene_root_->getOrCreateStateSet();
    osg::Point* point = dynamic_cast<osg::Point*>(state_set->getAttribute(osg::StateAttribute::POINT));
    if (point == NULL)
    return;

    if (point->getSize() >= 16.0)
    {
    point->setSize(16.0);
    return;
    }

    point->setSize(point->getSize()+1.0);

    return;
}

void OSGViewerWidget::decreasePointSize(void)
{
    osg::StateSet* state_Set = scene_root_->getOrCreateStateSet();
    osg::Point* point = dynamic_cast<osg::Point*>(state_Set->getAttribute(osg::StateAttribute::POINT));
    if (point == NULL)
    return;

    if (point->getSize() <= 1.0)
    {
    point->setSize(1.0);
    return;
    }

    point->setSize(point->getSize()-1.0);

    return;
}

void OSGViewerWidget::startRendering()
{
    addEventHandler(new osgGA::StateSetManipulator(getSceneData()->getOrCreateStateSet()));

    context()->moveToThread(&gl_thread_);
    threaded_painter_.moveToThread(&gl_thread_);
    connect(&gl_thread_, SIGNAL(started()), &threaded_painter_, SLOT(start()));
    gl_thread_.start();
}

void OSGViewerWidget::stopRendering()
{
    threaded_painter_.stop();
    gl_thread_.wait();
}

void OSGViewerWidget::paintGL(void)
{
    QMutexLocker locker(&mutex_);

    if (first_frame_)
    {
        first_frame_ = false;
        QMutexLocker locker_global(&mutex_first_frame_);
        frame();
    }
    else
    {
        frame();
    }

    return;
}

void OSGViewerWidget::paintEvent(QPaintEvent * /*event*/)
{
    // Handled by the GLThread.
}

void OSGViewerWidget::closeEvent(QCloseEvent *event)
{
    stopRendering();
    QGLWidget::closeEvent(event);
}

osg::BoundingSphere OSGViewerWidget::getBoundingSphere(void) const
{
    osgUtil::UpdateVisitor update_visitor;
    scene_root_->accept(update_visitor);

    osg::ComputeBoundsVisitor visitor;
    scene_root_->accept(visitor);

    return scene_root_->getBound();
}

osg::BoundingBox OSGViewerWidget::getBoundingBox(void) const
{
  osgUtil::UpdateVisitor update_visitor;
  scene_root_->accept(update_visitor);

  osg::ComputeBoundsVisitor visitor;
  scene_root_->accept(visitor);

  return visitor.getBoundingBox();
}

// camera position
// if center_scene_ is true, camera will look at the center of the graph scene,
// or camera will be put at the original position of World Coordinate which is the real camera position in our scanning device
// however, here I put the camera not at the (0,0,0) but have a small z-translation for nice look in the scene resulting in not total accurate
// results in visibility determination
void OSGViewerWidget::centerScene(void)
{
    QMutexLocker locker(&mutex_);

    osgGA::CameraManipulator* camera_manipulator = getCameraManipulator();
    osg::BoundingSphere bounding_sphere = getBoundingSphere();
    double radius = bounding_sphere.radius();

    if (!center_scene_)
    {
        camera_manipulator->setHomePosition(osg::Vec3(0.0, 0.0, bounding_sphere.center().z()-4.0*radius), 
            osg::Vec3(0.0, 0.0, bounding_sphere.center().z()), up_vector_);
        camera_manipulator->home(0);
    }
    else
    {
        osg::Vec3d eye_offset(0.0, 0.0, -4.0*radius);
        camera_manipulator->setHomePosition(bounding_sphere.center()+eye_offset, bounding_sphere.center(), up_vector_);
        camera_manipulator->home(0);
    }
    

    return;
}

void OSGViewerWidget::addSceneChild(osg::Node *child)
{
    QMutexLocker locker(&mutex_);

    scene_root_->addChild(child);

    return;
}

void OSGViewerWidget::addOtherChild(osg::Node *child)
{
    QMutexLocker locker(&mutex_);

    other_root_->addChild(child);

    return;
}

void OSGViewerWidget::removeSceneChild(osg::Node *child)
{
    QMutexLocker locker(&mutex_);

    scene_root_->removeChild(child);

    return;
}

void OSGViewerWidget::removeOtherChild(osg::Node *child)
{
    QMutexLocker locker(&mutex_);

    other_root_->removeChild(child);

    return;
}

void OSGViewerWidget::removeSceneChildren(void)
{
    QMutexLocker locker(&mutex_);

    if (scene_root_->getNumChildren() != 0)
        scene_root_->removeChildren(0, scene_root_->getNumChildren());

    return;
}

void OSGViewerWidget::replaceSceneChild(osg::Node* old_child, osg::Node* new_child)
{
    QMutexLocker locker(&mutex_);

    scene_root_->removeChild(old_child);
    scene_root_->addChild(new_child);

    return;
}

bool OSGViewerWidget::hasSceneChild(osg::Node *child)
{
    QMutexLocker locker(&mutex_);

    return (scene_root_->getChildIndex(child) != -1);
}

bool OSGViewerWidget::hasOtherChild(osg::Node *child)
{
    QMutexLocker locker(&mutex_);

    return (other_root_->getChildIndex(child) != -1);
}

void OSGViewerWidget::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_Equal)
    {
    if (event->modifiers() == Qt::CTRL)
    {
        increasePointSize();
    }
    else if (event->modifiers() == Qt::SHIFT)
    {
        increaseLineWidth();
    }
    }                                                   
    else if (event->key() == Qt::Key_Minus)
    {
    if (event->modifiers() == Qt::CTRL)
    {
        decreasePointSize();
    }
    else if (event->modifiers() == Qt::SHIFT)
    {
        decreaseLineWidth();
    }
    }
    else if (event->key() == Qt::Key_8)
    {
    centerScene();
    }

    AdapterWidget::keyPressEvent(event);

    return;
}

void OSGViewerWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
    QMenu menu(this);

    menu.exec(event->globalPos());

    AdapterWidget::mousePressEvent(event);  

    return;
}


CameraHandler::CameraHandler()
    :count_(0){}

CameraHandler::~CameraHandler(void)
{

}

bool CameraHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
    switch(ea.getEventType())
    {
    case(osgGA::GUIEventAdapter::KEYDOWN):
        {
            osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
            if (view == NULL)
                return false;

            switch (ea.getKey())
            {
            case (osgGA::GUIEventAdapter::KEY_C):
                {
                    osgGA::CameraManipulator* cm = view->getCameraManipulator();
                    osg::Vec3d eye, center, up;
                    cm->getHomePosition(eye, center, up);

                    if (count_ == 0)
                    {
                        eye_  = eye;
                        center_ = center;
                        up_ = up;
                    }

                    double angle = (2*M_PI) / 6; // default six cameras in our case
                    Registrator* registrator = MainWindow::getInstance()->getRegistrator();
                    osg::Vec3 axis_normal = registrator->getAxisNormal();
                    osg::Vec3 center_pivot = registrator->getPivotPoint();

                    osg::Matrix rotation = osg::Matrix::rotate(-angle, axis_normal);
                    osg::Vec3d new_eye = center + rotation.preMult(eye - center);
                    osg::Vec3d new_up = rotation.preMult(up);
                    cm->setHomePosition(new_eye, center, new_up);
                    cm->home(0);

                    ++ count_;
                    
                    break;
                }
            default:
                break;
            }

            switch (ea.getModKeyMask())
            {
            case (osgGA::GUIEventAdapter::MODKEY_ALT):
                {
                    if (ea.getKey() == osgGA::GUIEventAdapter::KEY_C)
                    {
                        osgGA::CameraManipulator* cm = view->getCameraManipulator();
                        cm->setHomePosition(eye_, center_, up_);
                        cm->home(0);

                        break;
                    }
                }

            default:
                break;
            }
        }
  
        break;
    }

    return false;
}

void OSGViewerWidget::setCenterScene(bool center_scene)
{
    center_scene_ = center_scene;
}