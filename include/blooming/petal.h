#ifndef PETAL_H
#define PETAL_H

#include "point_cloud.h"
#include "mesh_model.h"

class Petal: public PointCloud, public MeshModel
{
public:
    Petal(int id);
    Petal(Petal& petal);
    virtual ~Petal();

    void setMeshModel(MeshModel* mesh_model);
    void setPointCloud(PointCloud* point_cloud);

    inline int getPetalId() { return petal_id_; }

protected:
    virtual void updateImpl();
    virtual void visualizePetal();

    void pickEvent(int pick_mode, osg::Vec3 position){}

private:
    int     petal_id_;
};

typedef std::vector<osg::ref_ptr<Petal> >  Petals;

#endif