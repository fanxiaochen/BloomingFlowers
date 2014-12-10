#ifndef PETAL_H
#define PETAL_H

#include "point_cloud.h"

class Petal: public PointCloud
{
public:

    inline int getPetalId() { return petal_id_; }

private:
    int     petal_id_;
};

#endif