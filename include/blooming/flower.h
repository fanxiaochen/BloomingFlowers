#ifndef FLOWER_H
#define FLOWER_H

#include "point_cloud.h"
#include "mesh_model.h"
#include "petal.h"

class Flower: public PointCloud, public MeshModel
{
public:
    Flower();
    virtual ~Flower();

protected:
    virtual void clearData();
    virtual void updateImpl();
    virtual void visualizeFlower();

    void pickEvent(int pick_mode, osg::Vec3 position){}

private:
    Petals            petals_;
};

typedef std::vector<Flower> Flowers;

#endif 