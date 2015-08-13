#include "collision_detector.h"
#include <set>
#include <math.h>
#define FABS(x) (float(fabs(x)))        /* implement as is fastest on your machine */

/* if USE_EPSILON_TEST is true then we do a check:
         if |dv|<EPSILON then dv=0.0;
   else no check is done (which is less robust)
*/
#define USE_EPSILON_TEST TRUE
#define EPSILON 0.000001


/* some macros */
#define CROSS(dest,v1,v2){                     \
              dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
              dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
              dest[2]=v1[0]*v2[1]-v1[1]*v2[0];}

#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])

#define SUB(dest,v1,v2){         \
            dest[0]=v1[0]-v2[0]; \
            dest[1]=v1[1]-v2[1]; \
            dest[2]=v1[2]-v2[2];}

/* sort so that a<=b */
#define SORT(a,b)       \
             if(a>b)    \
             {          \
               float c; \
               c=a;     \
               a=b;     \
               b=c;     \
             }


/* this edge to edge test is based on Franlin Antonio's gem:
   "Faster Line Segment Intersection", in Graphics Gems III,
   pp. 199-202 */
#define EDGE_EDGE_TEST(V0,U0,U1)                      \
  Bx=U0[i0]-U1[i0];                                   \
  By=U0[i1]-U1[i1];                                   \
  Cx=V0[i0]-U0[i0];                                   \
  Cy=V0[i1]-U0[i1];                                   \
  f=Ay*Bx-Ax*By;                                      \
  d=By*Cx-Bx*Cy;                                      \
  if((f>0 && d>=0 && d<=f) || (f<0 && d<=0 && d>=f))  \
  {                                                   \
    e=Ax*Cy-Ay*Cx;                                    \
    if(f>0)                                           \
    {                                                 \
      if(e>=0 && e<=f) return 1;                      \
    }                                                 \
    else                                              \
    {                                                 \
      if(e<=0 && e>=f) return 1;                      \
    }                                                 \
  }

#define EDGE_AGAINST_TRI_EDGES(V0,V1,U0,U1,U2) \
{                                              \
  float Ax,Ay,Bx,By,Cx,Cy,e,d,f;               \
  Ax=V1[i0]-V0[i0];                            \
  Ay=V1[i1]-V0[i1];                            \
  /* test edge U0,U1 against V0,V1 */          \
  EDGE_EDGE_TEST(V0,U0,U1);                    \
  /* test edge U1,U2 against V0,V1 */          \
  EDGE_EDGE_TEST(V0,U1,U2);                    \
  /* test edge U2,U1 against V0,V1 */          \
  EDGE_EDGE_TEST(V0,U2,U0);                    \
}

#define POINT_IN_TRI(V0,U0,U1,U2)           \
{                                           \
  float a,b,c,d0,d1,d2;                     \
  /* is T1 completly inside T2? */          \
  /* check if V0 is inside tri(U0,U1,U2) */ \
  a=U1[i1]-U0[i1];                          \
  b=-(U1[i0]-U0[i0]);                       \
  c=-a*U0[i0]-b*U0[i1];                     \
  d0=a*V0[i0]+b*V0[i1]+c;                   \
                                            \
  a=U2[i1]-U1[i1];                          \
  b=-(U2[i0]-U1[i0]);                       \
  c=-a*U1[i0]-b*U1[i1];                     \
  d1=a*V0[i0]+b*V0[i1]+c;                   \
                                            \
  a=U0[i1]-U2[i1];                          \
  b=-(U0[i0]-U2[i0]);                       \
  c=-a*U2[i0]-b*U2[i1];                     \
  d2=a*V0[i0]+b*V0[i1]+c;                   \
  if(d0*d1>0.0)                             \
  {                                         \
    if(d0*d2>0.0) return 1;                 \
  }                                         \
}

int coplanar_tri_tri(float N[3],float V0[3],float V1[3],float V2[3],
                     float U0[3],float U1[3],float U2[3])
{
   float A[3];
   short i0,i1;
   /* first project onto an axis-aligned plane, that maximizes the area */
   /* of the triangles, compute indices: i0,i1. */
   A[0]=FABS(N[0]);
   A[1]=FABS(N[1]);
   A[2]=FABS(N[2]);
   if(A[0]>A[1])
   {
      if(A[0]>A[2])
      {
          i0=1;      /* A[0] is greatest */
          i1=2;
      }
      else
      {
          i0=0;      /* A[2] is greatest */
          i1=1;
      }
   }
   else   /* A[0]<=A[1] */
   {
      if(A[2]>A[1])
      {
          i0=0;      /* A[2] is greatest */
          i1=1;
      }
      else
      {
          i0=0;      /* A[1] is greatest */
          i1=2;
      }
    }

    /* test all edges of triangle 1 against the edges of triangle 2 */
    EDGE_AGAINST_TRI_EDGES(V0,V1,U0,U1,U2);
    EDGE_AGAINST_TRI_EDGES(V1,V2,U0,U1,U2);
    EDGE_AGAINST_TRI_EDGES(V2,V0,U0,U1,U2);

    /* finally, test if tri1 is totally contained in tri2 or vice versa */
    POINT_IN_TRI(V0,U0,U1,U2);
    POINT_IN_TRI(U0,V0,V1,V2);

    return 0;
}



#define NEWCOMPUTE_INTERVALS(VV0,VV1,VV2,D0,D1,D2,D0D1,D0D2,A,B,C,X0,X1) \
{ \
        if(D0D1>0.0f) \
        { \
                /* here we know that D0D2<=0.0 */ \
            /* that is D0, D1 are on the same side, D2 on the other or on the plane */ \
                A=VV2; B=(VV0-VV2)*D2; C=(VV1-VV2)*D2; X0=D2-D0; X1=D2-D1; \
        } \
        else if(D0D2>0.0f)\
        { \
                /* here we know that d0d1<=0.0 */ \
            A=VV1; B=(VV0-VV1)*D1; C=(VV2-VV1)*D1; X0=D1-D0; X1=D1-D2; \
        } \
        else if(D1*D2>0.0f || D0!=0.0f) \
        { \
                /* here we know that d0d1<=0.0 or that D0!=0.0 */ \
                A=VV0; B=(VV1-VV0)*D0; C=(VV2-VV0)*D0; X0=D0-D1; X1=D0-D2; \
        } \
        else if(D1!=0.0f) \
        { \
                A=VV1; B=(VV0-VV1)*D1; C=(VV2-VV1)*D1; X0=D1-D0; X1=D1-D2; \
        } \
        else if(D2!=0.0f) \
        { \
                A=VV2; B=(VV0-VV2)*D2; C=(VV1-VV2)*D2; X0=D2-D0; X1=D2-D1; \
        } \
        else \
        { \
                /* triangles are coplanar */ \
                return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2); \
        } \
}



int CollisionDetector::noDivTriTriIsect(float V0[3],float V1[3],float V2[3],
                     float U0[3],float U1[3],float U2[3])
{
  float E1[3],E2[3];
  float N1[3],N2[3],d1,d2;
  float du0,du1,du2,dv0,dv1,dv2;
  float D[3];
  float isect1[2], isect2[2];
  float du0du1,du0du2,dv0dv1,dv0dv2;
  short index;
  float vp0,vp1,vp2;
  float up0,up1,up2;
  float bb,cc,max;

  /* compute plane equation of triangle(V0,V1,V2) */
  SUB(E1,V1,V0);
  SUB(E2,V2,V0);
  CROSS(N1,E1,E2);
  d1=-DOT(N1,V0);
  /* plane equation 1: N1.X+d1=0 */

  /* put U0,U1,U2 into plane equation 1 to compute signed distances to the plane*/
  du0=DOT(N1,U0)+d1;
  du1=DOT(N1,U1)+d1;
  du2=DOT(N1,U2)+d1;

  /* coplanarity robustness check */
#if USE_EPSILON_TEST==TRUE
  if(FABS(du0)<EPSILON) du0=0.0;
  if(FABS(du1)<EPSILON) du1=0.0;
  if(FABS(du2)<EPSILON) du2=0.0;
#endif
  du0du1=du0*du1;
  du0du2=du0*du2;

  if(du0du1>0.0f && du0du2>0.0f) /* same sign on all of them + not equal 0 ? */
    return 0;                    /* no intersection occurs */

  /* compute plane of triangle (U0,U1,U2) */
  SUB(E1,U1,U0);
  SUB(E2,U2,U0);
  CROSS(N2,E1,E2);
  d2=-DOT(N2,U0);
  /* plane equation 2: N2.X+d2=0 */

  /* put V0,V1,V2 into plane equation 2 */
  dv0=DOT(N2,V0)+d2;
  dv1=DOT(N2,V1)+d2;
  dv2=DOT(N2,V2)+d2;

#if USE_EPSILON_TEST==TRUE
  if(FABS(dv0)<EPSILON) dv0=0.0;
  if(FABS(dv1)<EPSILON) dv1=0.0;
  if(FABS(dv2)<EPSILON) dv2=0.0;
#endif

  dv0dv1=dv0*dv1;
  dv0dv2=dv0*dv2;

  if(dv0dv1>0.0f && dv0dv2>0.0f) /* same sign on all of them + not equal 0 ? */
    return 0;                    /* no intersection occurs */

  /* compute direction of intersection line */
  CROSS(D,N1,N2);

  /* compute and index to the largest component of D */
  max=(float)FABS(D[0]);
  index=0;
  bb=(float)FABS(D[1]);
  cc=(float)FABS(D[2]);
  if(bb>max) max=bb,index=1;
  if(cc>max) max=cc,index=2;

  /* this is the simplified projection onto L*/
  vp0=V0[index];
  vp1=V1[index];
  vp2=V2[index];

  up0=U0[index];
  up1=U1[index];
  up2=U2[index];

  /* compute interval for triangle 1 */
  float a,b,c,x0,x1;
  NEWCOMPUTE_INTERVALS(vp0,vp1,vp2,dv0,dv1,dv2,dv0dv1,dv0dv2,a,b,c,x0,x1);

  /* compute interval for triangle 2 */
  float d,e,f,y0,y1;
  NEWCOMPUTE_INTERVALS(up0,up1,up2,du0,du1,du2,du0du1,du0du2,d,e,f,y0,y1);

  float xx,yy,xxyy,tmp;
  xx=x0*x1;
  yy=y0*y1;
  xxyy=xx*yy;

  tmp=a*xxyy;
  isect1[0]=tmp+b*x1*yy;
  isect1[1]=tmp+c*x0*yy;

  tmp=d*xxyy;
  isect2[0]=tmp+e*xx*y1;
  isect2[1]=tmp+f*xx*y0;

  SORT(isect1[0],isect1[1]);
  SORT(isect2[0],isect2[1]);

  if(isect1[1]<isect2[0] || isect2[1]<isect1[0]) return 0;
  return 1;
}

void CollisionDetector::setFlower( Flower* flower )
{
	flower_ = flower;
}


CollisionDetector::CollisionDetector()
{

}


void CollisionDetector::checkCollision( int a_id, int b_id )
{
	Petal& petal_a = flower_->getPetals().at(a_id);
	Petal& petal_b = flower_->getPetals().at(b_id);

	typedef  std::vector<int> TriFacet;
	std::vector< TriFacet >& triangles_a = petal_a.getFaces(); 
	std::vector< TriFacet >& triangles_b = petal_b.getFaces();

	osg::ref_ptr<osg::Vec3Array> vertices_a = petal_a.getVertices();
	osg::ref_ptr<osg::Vec3Array> vertices_b = petal_b.getVertices();

	float V0[3],V1[3],V2[3],U0[3],U1[3],U2[3];
	for( int i = 0; i!= triangles_a.size(); ++i )
	{
		TriFacet& t1 = triangles_a[i];
		osg::Vec3 tv0 = vertices_a->at(t1[0]);
		osg::Vec3 tv1 = vertices_a->at(t1[1]);
		osg::Vec3 tv2 = vertices_a->at(t1[2]);
		V0[0] = tv0.x(); V0[1] = tv0.y(); V0[2] = tv0.z();
		V1[0] = tv1.x(); V1[1] = tv1.y(); V1[2] = tv1.z();
		V2[0] = tv2.x(); V2[1] = tv2.y(); V2[2] = tv2.z();
		for( int j = 0; j!= triangles_b.size(); ++j )
		{
			TriFacet& t2 = triangles_b[j];
			osg::Vec3 tu0 = vertices_b->at(t2[0]);
			osg::Vec3 tu1 = vertices_b->at(t2[1]);
			osg::Vec3 tu2 = vertices_b->at(t2[2]);
			U0[0] = tu0.x(); U0[1] = tu0.y(); U0[2] = tu0.z();
			U1[0] = tu1.x(); U1[1] = tu1.y(); U1[2] = tu1.z();
			U2[0] = tu2.x(); U2[1] = tu2.y(); U2[2] = tu2.z();
			
			int isIntersected = noDivTriTriIsect( V0,V1,V2, U0,U1,U2);

			if( isIntersected )
			{
				FacetPair p;
				p.first = std::make_pair( a_id, i);
				p.second = std::make_pair( b_id, j);
				intersected_pairs_.push_back( p );
			}
				
		}
	}

}

void CollisionDetector::checkCollision()
{
	intersected_pairs_.clear();
	int petal_num = flower_->getPetals().size();
	for( int a_id = 0; a_id != petal_num; ++a_id )
	{
		for( int b_id = a_id+1; b_id < petal_num; ++b_id )
		{
			checkCollision(a_id, b_id);
		}
	}
	// Put the intersection information to meshModel for visualization.
	Petals& petals = flower_->getPetals(); 
	std::vector<std::set<int>> sets( petal_num, std::set<int>());
	for( int i = 0; i != intersected_pairs_.size(); ++i )
	{
		FacetPair& p = intersected_pairs_[i];
		sets[p.first.first].insert( p.first.second );
		sets[p.second.first].insert( p.second.second);		
	}
	for( int i= 0 ;i != petal_num; ++i )
	{
		petals[i].getIntersectedTriangles() = std::vector<int>( sets[i].begin(), sets[i].end());
	}

}

void CollisionDetector::resolveCollision()
{
	PetalRelation& petal_relation = flower_->getPetalRelation();
	// check for the colliding point 
	std::map<VertexIndex, CollidingPoint> colliding_points;
	for( int i = 0; i != intersected_pairs_.size(); ++i )
	{
		FacetPair p = intersected_pairs_[i];
		if( petal_relation(p.first.first, p.second.first) == -1)
		{
			p.swap(intersected_pairs_[i]);
		}
	}
	
}

