
#ifndef FITTING_ERROR_MEASURER_H
#define FITTING_ERROR_MEASURER_H

#include <osg/Vec3>
#include <vector>


class Flower;
class PointCloud;

struct PointTriPair
{
	int				point_id;    // the index of the require point
	osg::Vec3		p_;    // position of the require point 

	int				petel_id_;   // the index of the petal
	int				tri_id_;	 // the index of the nearest triangle
	osg::Vec3		closest_p_;	// the nearest point inside the triangle to the require point
	float			dis2_;       // the distance           
}; 

class FittingErrorMeasurer
{
public:
	void setFlowerAndPointCloud( Flower* ptr_flower, PointCloud* ptr_cloud );

	void computeDisFromCloud2Flower();

	float getMeanDis();

private:
	void comptueNearestTriangle(osg::Vec3 p, int petal_id);

private:
	Flower*						  ptr_flower_;
	PointCloud*					  ptr_cloud_;
	std::vector<PointTriPair>     point_tri_pairs_;
};


#endif