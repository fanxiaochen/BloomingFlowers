#ifndef FLOWER_H
#define FLOWER_H

#include "renderable.h"
#include "petal.h"

class Flower
{
public:
    Flower();
    virtual ~Flower();

    inline Petals& getPetals(){ return petals_; }

    void show();
    void update();

    void deform(const std::vector<osg::ref_ptr<osg::Vec3Array> >& pos, 
        const std::vector<std::vector<int> > idx);

    Flower simplifyMesh(int delta);
    void searchNearestIdx(Flower& simplified_flower, 
        std::vector<std::vector<int> >& knn_idx);

private:
    Petals            petals_;

    bool              show_points_;
    bool              show_mesh_;
};

typedef std::vector<Flower> Flowers;

#endif 