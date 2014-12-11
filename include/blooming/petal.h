#ifndef PETAL_H
#define PETAL_H

#include "point_cloud.h"
#include "mesh_model.h"

class Petal: public PointCloud, public MeshModel
{
public:
    Petal(int id);
    virtual ~Petal();

    inline int getPetalId() { return petal_id_; }

protected:
    virtual void updateImpl();
    virtual void visualizePetal();

private:
    int     petal_id_;
};

#endif