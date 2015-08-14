#ifndef COLLISION_DETECTOR_H
#define COLLISION_DETECTOR_H

#include "flower.h"

class CollisionDetector
{
	typedef std::pair<int,int> FacetIndex;
	typedef std::pair<int,int> VertexIndex;
	typedef std::pair<FacetIndex, FacetIndex>  FacetPair;

public:
	CollisionDetector();
	void setFlower(Flower* flower);
	
	void checkCollision();

	void resolveCollision();

private:
	void checkCollision( int a_id, int b_id );

	/* Triangle/triangle intersection test routine,
	* by Tomas Moller, 1997.
	* See article "A Fast Triangle-Triangle Intersection Test",
	* Journal of Graphics Tools, 2(2), 1997
	*
	* Updated June 1999: removed the divisions -- a little faster now!
	* Updated October 1999: added {} to CROSS and SUB macros 
	*
	* int NoDivTriTriIsect(float V0[3],float V1[3],float V2[3],
	*                      float U0[3],float U1[3],float U2[3])
	*
	* parameters: vertices of triangle 1: V0,V1,V2
	*             vertices of triangle 2: U0,U1,U2
	* result    : returns 1 if the triangles intersect, otherwise 0
	*
	*/
	int noDivTriTriIsect( float V0[3], float V1[3], float V2[3],
		float U0[3], float U1[3], float U2[3]);

private:
	Flower*						  flower_;
	std::vector<FacetPair>        intersected_pairs_;
	std::vector<CollidingPoint>   colliding_points_;

};






#endif