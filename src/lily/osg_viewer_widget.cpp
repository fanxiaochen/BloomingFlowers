#include <QFileDialog>
#include <QTextStream>
#include <QResizeEvent>

#include <osg/Depth>
#include <osg/Point>
#include <osg/LineWidth>
#include <osg/ComputeBoundsVisitor>
#include <osgGA/TrackballManipulator>

#include "main_window.h"
#include "toggle_handler.h"

#include "osg_viewer_widget.h"


QMutex OSGViewerWidget::mutex_first_frame_;

OSGViewerWidget::OSGViewerWidget(QWidget * parent, const QGLWidget * shareWidget, Qt::WindowFlags f)
	:AdapterWidget(parent, shareWidget, f, true),
	scene_root_(new osg::Group),
	other_root_(new osg::Group),
	gl_thread_(this),
	threaded_painter_(this),
	first_frame_(true),
	up_vector_(0.0f, 1.0f, 0.0f)
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
	//addEventHandler(new StateSetManipulator(getSceneData()->getOrCreateStateSet()));

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
	return scene_root_->getBound();
}

void OSGViewerWidget::centerScene(void)
{
	/*QMutexLocker locker(&mutex_);

	osg::BoundingBox bounding_box = getBoundingBox();
	if (bounding_box.xMax() == -std::numeric_limits<float>::max())
	{
	bounding_box._min = osg::Vec3(0.0, 0.0, 0.0);
	bounding_box._max = osg::Vec3(1.0, 1.0, 1.0);
	}

	osg::Vec3 center = (bounding_box._min+bounding_box._max)/2;
	double distance = (bounding_box._max-bounding_box._min).length();
	osg::Vec3d eye_offset(1.0, 1.0, -1.0);
	eye_offset.normalize();
	eye_offset = eye_offset*distance*0.9;

	osgGA::CameraManipulator* camera_manipulator = getCameraManipulator();
	camera_manipulator->setHomePosition(center+eye_offset, center, up_vector_);
	camera_manipulator->home(0);*/

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