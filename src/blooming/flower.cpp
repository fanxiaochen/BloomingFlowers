
#include <osg/Geometry>
#include <osg/ShapeDrawable>
#include <osg/Point>

#include "flower.h"

Flower::Flower()
    :show_points_(false), show_mesh_(false)
{
}

Flower::~Flower()
{}

void Flower::clearData()
{
    petals_.clear();
}

void Flower::updateImpl()
{
    visualizeFlower();
    return;
}

void Flower::visualizeFlower()
{
    /*osg::ref_ptr<osg::Vec3Array>  vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec4Array>  colors = new osg::Vec4Array;

    for (size_t i = 0, i_end = size(); i < i_end; i ++)
    {
        if (!segmented_)
        {
            const Point& point = at(i);
            vertices->push_back(osg::Vec3(point.x, point.y, point.z));
            colors->push_back(osg::Vec4(point.r / 255.0, point.g / 255.0, point.b / 255.0, 0));
        }
        else 
        {
            const FlowerPoint& point = flower_points_.at(i);
            vertices->push_back(osg::Vec3(point._pt.x, point._pt.y, point._pt.z));
            osg::Vec4 color = ColorMap::getInstance().getDiscreteColor(point._label + 1);
            colors->push_back(color);
        }
    }

    osg::Geometry* geometry = new osg::Geometry;
    geometry->setVertexArray(vertices);
    geometry->setColorArray(colors);
    geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, size()));
    geometry->getOrCreateStateSet()->setAttribute(new osg::Point(5.0f));
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable(geometry);
    content_root_->addChild(geode);


    for (auto& it = picked_points_.begin(); it != picked_points_.end(); it ++)
    {
        osg::Geode* picked_geode = new osg::Geode();
        osg::Sphere* sphere = new osg::Sphere(*it, 5.0);
        osg::ShapeDrawable* drawable = new osg::ShapeDrawable(sphere);
        drawable->setColor(osg::Vec4(1.0,0.0,0.0,0.0));
        picked_geode->addDrawable(drawable);
        content_root_->addChild(picked_geode);
    }

    return;*/
}



