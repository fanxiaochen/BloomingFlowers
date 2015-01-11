#ifndef FLOWER_H
#define FLOWER_H

#include "renderable.h"
#include "mesh_model.h"

typedef MeshModel   Petal;
typedef std::vector<Petal>  Petals;

class Flower
{
public:
    Flower();
    Flower(const Flower& flower); // deep copy
    virtual ~Flower(); 

    inline const Petals& getPetals() const { return petals_; }
    inline Petals& getPetals() { return petals_; }

    void save(const std::string& flower_path);

    void show();
    void update();
    void hide();

    void rotate(const osg::Matrix& rot_matrix);

    void initVisibility();
    void determineVisibility();

private:
    int contains(Petal* petal);

    void determineIntersection();
    
private:
    Petals            petals_;

};

typedef std::vector<Flower> Flowers;

#endif 