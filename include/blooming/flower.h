#ifndef FLOWER_H
#define FLOWER_H

#include "renderable.h"
#include "mesh_model.h"

typedef MeshModel   Petal;
typedef std::vector<Petal>  Petals;

// Flower could not be template of osg::ref_ptr
class Flower
{
public:
    Flower();
    Flower(const Flower& flower); // deep copy
    virtual ~Flower(); 

    inline const Petals& getPetals() const { return petals_; }
    inline Petals& getPetals() { return petals_; }

    void save(const std::string& flower_path);
    void save(const std::string& flower_folder, int frame);
    void load(const std::string& flower_path);

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

//class FlowersManager
//{
//public:
//    FlowersManager();
//    virtual ~FlowersManager();
//
//    void setFlowersFolder(const std::string& flowers_folder);
//
//    Flower getFlower(int frame);
//    Flower next(int frame);
//    Flower previous(int frame);
//
//private:
//    std::string flowers_folder_;
//};

#endif 