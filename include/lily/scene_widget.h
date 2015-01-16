
#ifndef SCENE_WIDGET_H_
#define SCENE_WIDGET_H_

#include <osg/LightSource>

#include "osg_viewer_widget.h"

class SceneWidget : public OSGViewerWidget
{
  Q_OBJECT

public:
	SceneWidget(QWidget * parent = 0, const QGLWidget * shareWidget = 0, Qt::WindowFlags f = 0);
	virtual ~SceneWidget();

	virtual QSize sizeHint() const {return QSize(256, 256);}

    public slots:
    bool setLight();

private:
    void turnOnLights();
    void turnOffLights();

private:
    std::vector<osg::ref_ptr<osg::LightSource> > light_sources_;
    int light_number_;
    bool light_status_;
};

#endif // SCENE_WIDGET_H_
