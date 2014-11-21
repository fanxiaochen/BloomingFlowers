#ifndef POINT_INTERSECTOR_H_
#define POINT_INTERSECTOR_H_

#include <osgUtil/LineSegmentIntersector>

class PointIntersector : public osgUtil::LineSegmentIntersector
{
public:
    PointIntersector();
    PointIntersector( const osg::Vec3& start, const osg::Vec3& end );
    PointIntersector( CoordinateFrame cf, double x, double y );
    
    void setPickBias( float bias ) { _pick_bias = bias; }
    float getPickBias() const { return _pick_bias; }
    
    virtual Intersector* clone( osgUtil::IntersectionVisitor& iv );
    virtual void intersect( osgUtil::IntersectionVisitor& iv, osg::Drawable* drawable );
    
protected:
    virtual ~PointIntersector() {}
    float _pick_bias;
};


template <class T>
T* computeIntersection(osgViewer::View* view,
                       const osgGA::GUIEventAdapter& ea,
                       osgUtil::LineSegmentIntersector::Intersection& intersection,
                       osg::NodePath& node_path)
{
    T* intersection_object = NULL;

    osgUtil::LineSegmentIntersector::Intersections intersections;

    float x = ea.getX();
    float y = ea.getY();

    if (!view->computeIntersections(x,y,intersections)) {
        return intersection_object;
    }

    intersection = *(intersections.begin());
    node_path = intersection.nodePath;

    while (!node_path.empty()) {
        intersection_object = dynamic_cast<T*>(node_path.back());
        if (intersection_object != NULL) {
            break;
        }
        node_path.pop_back();
    }

    return intersection_object;
}

template <class T>
T* computePointIntersection(osgViewer::View* view,
                            const osgGA::GUIEventAdapter& ea,
                            osgUtil::LineSegmentIntersector::Intersection& intersection,
                            osg::NodePath& node_path)
{
    T* intersection_object = NULL;

    float x = ea.getX();
    float y = ea.getY();

    osg::ref_ptr<PointIntersector> intersector = new PointIntersector(osgUtil::Intersector::WINDOW, x, y);
    osgUtil::IntersectionVisitor intersection_visitor( intersector.get() );
    view->getCamera()->accept( intersection_visitor );

    if (!intersector->containsIntersections())
        return intersection_object;

    intersection = *(intersector->getIntersections().begin());
    node_path = intersection.nodePath;

    while (!node_path.empty()) {
        intersection_object = dynamic_cast<T*>(node_path.back());
        if (intersection_object != NULL) {
            break;
        }
        node_path.pop_back();
    }

    return intersection_object;
}

#endif //POINT_INTERSECTOR_H_
