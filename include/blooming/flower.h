#ifndef FLOWER_H
#define FLOWER_H

#include "renderable.h"
#include "petal.h"

class Flower: public Renderable
{
public:
    Flower();
    virtual ~Flower();

    inline Petals& getPetals(){ return petals_; }

protected:
    virtual void clearData();
    virtual void updateImpl();
    virtual void visualizeFlower();

    void pickEvent(int pick_mode, osg::Vec3 position){}

private:
    Petals            petals_;

    bool              show_points_;
    bool              show_mesh_;
};

typedef std::vector<osg::ref_ptr<Flower> > Flowers;

#endif 