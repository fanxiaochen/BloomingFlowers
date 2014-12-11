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

    Flower simplifyMesh(int delta);


private:
    Petals            petals_;

    bool              show_points_;
    bool              show_mesh_;
};

typedef std::vector<Flower> Flowers;

#endif 