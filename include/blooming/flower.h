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

    void deform(const std::vector<osg::ref_ptr<osg::Vec3Array> >& pos, 
        const std::vector<std::vector<int> > idx);

    void deform(const std::vector<osg::ref_ptr<osg::Vec3Array> >& hard_ctrs, const std::vector<std::vector<int> > hard_idx, 
        const std::vector<osg::ref_ptr<osg::Vec3Array> >& soft_ctrs, const std::vector<std::vector<int> > soft_idx);

    Flower simplifyMesh(int scale);

    void buildHardCtrs(int scale);

    void determineVisibility();

private:
    int contains(Petal* petal);

    void determineIntersection();
    
    //// source is simplified_flower, target is this flower
    //void searchNearestIdx(Flower& simplified_flower, 
    //    std::vector<std::vector<int> >& knn_idx);

    ////source is this flower, target is point_cloud
    //void searchNearestIdx(PointCloud& point_cloud,
    //    std::vector<osg::ref_ptr<osg::Vec3Array> >& knn_pos);

private:
    Petals            petals_;

};

typedef std::vector<Flower> Flowers;

#endif 