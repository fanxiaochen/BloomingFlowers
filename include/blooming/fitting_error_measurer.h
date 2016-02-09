
#ifndef FITTING_ERROR_MEASURER_H
#define FITTING_ERROR_MEASURER_H

#include <osg/Vec3>
#include <vector>


class Flower;
class PointCloud;

class PointTriPair
{
public:
	PointTriPair()
	{
		point_id_ = -1;
		p_ = osg::Vec3(0, 0, 0);

		petel_id_ = -1;
		tri_id_ = -1;
		closest_p_ = osg::Vec3(0, 0, 0);
		dis2_ = 0;
	}



public:
	int				point_id_;    // the index of the require point
	osg::Vec3		p_;    // position of the require point 

	int				petel_id_;   // the index of the petal
	int				tri_id_;	 // the index of the nearest triangle
	osg::Vec3		closest_p_;	// the nearest point inside the triangle to the require point
	double			dis2_;       // the distance           
}; 

class FittingErrorMeasurer
{
public:
	void setFlowerAndPointCloud( Flower* ptr_flower, PointCloud* ptr_cloud );

	void computeDisFromCloud2Flower();

	double getMeanDis();

private:
	void comptueNearestTriangle(int point_id);
	PointTriPair comptueNearestTriangle(osg::Vec3 p, int petal_id);

private:
	Flower*						  ptr_flower_;
	PointCloud*					  ptr_cloud_;
	std::vector<PointTriPair>     point_tri_pairs_;
	double						  err_;
};


#endif