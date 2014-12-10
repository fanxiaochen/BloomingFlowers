#ifndef FLOWER_H
#define FLOWER_H

#include "point_cloud.h"
#include "petal.h"

class Flower: public PointCloud
{
public:
    struct FlowerPoint
    {
        int _label;
        Point _pt;

        FlowerPoint()
        {
            _pt.x = 0;
            _pt.y = 0;
            _pt.z = 0;

            _label = -1;
        }

        FlowerPoint& operator + (const FlowerPoint& point)
        {
            this->_pt.x = this->_pt.x + point._pt.x;
            this->_pt.y = this->_pt.y + point._pt.y;
            this->_pt.z = this->_pt.z + point._pt.z;

            return *this;
        }

        FlowerPoint& operator - (const FlowerPoint& point)
        {
            this->_pt.x = this->_pt.x - point._pt.x;
            this->_pt.y = this->_pt.y - point._pt.y;
            this->_pt.z = this->_pt.z - point._pt.z;

            return *this;
        }

        FlowerPoint& operator * (float mul)
        {
            this->_pt.x = this->_pt.x * mul;
            this->_pt.y = this->_pt.y * mul;
            this->_pt.z = this->_pt.z * mul;

            return *this;
        }

        FlowerPoint& operator / (float div)
        {
            this->_pt.x = this->_pt.x / div;
            this->_pt.y = this->_pt.y / div;
            this->_pt.z = this->_pt.z / div;

            return *this;
        }

        FlowerPoint& operator = (const FlowerPoint& point)
        {
            this->_pt.x = point._pt.x;
            this->_pt.y = point._pt.y;
            this->_pt.z = point._pt.z;

            this->_label = point._label;

            return *this;
        }

        bool operator == (const FlowerPoint& point)
        {
            const float eps = 1e-3;

            if (fabs(this->_pt.x - point._pt.x) < eps&&
                fabs(this->_pt.y - point._pt.y) < eps&&
                fabs(this->_pt.z - point._pt.z) < eps)
                return true;
            else
                return false;
        }
    };

public:
    Flower();
    virtual ~Flower();

    inline std::vector<FlowerPoint>& getFlowerPoints() { return flower_points_; }

    // segmentation
    inline const std::vector<int>& getPickedIndices() const { return picked_indices_; }
    inline std::vector<osg::Vec3>& getPickedPoints() { return picked_points_; }
    virtual void pickEvent(int pick_mode, osg::Vec3 position);
    void petal_segmentation();
    void resetSegmentation();

protected:
    virtual void clearData();
    virtual void visualizePoints();

private:

    // segmentation
    void k_means();
    float distance(const FlowerPoint& p1, const FlowerPoint& p2);
    void estimateNormals();
    void computeFlowerPoints();
    void setCenters();
    void updateCenters();
    int determineCluster(const FlowerPoint& point);
    FlowerPoint mean_center(const std::vector<int>& ids);
    bool terminal(const std::vector<FlowerPoint>& cluster_centers, const std::vector<FlowerPoint>& next_centers);

private:
    std::vector<Petal>            petals_;

    // segmentation
    std::vector<FlowerPoint>      flower_points_;

    std::vector<int>              picked_indices_;
    std::vector<osg::Vec3>        picked_points_;
    std::vector<FlowerPoint>      cluster_centers_;

    bool                          segmented_;

};

#endif 