#ifndef PETAL_CLOUD_H
#define PETAL_CLOUD_H

#include "point_cloud.h"

class PetalCloud: public PointCloud
{
public:

    inline int getPetalId() { return petal_id_; }

private:
    int     petal_id_;
};

#endif