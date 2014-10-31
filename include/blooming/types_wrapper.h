#ifndef TYPES_WRAPPER_H
#define TYPES_WRAPPER_H

#include "cpd/base/matrix.hpp"

#include "point_cloud.h"

typedef cpd::MatrixType<float, 3>::MatrixD PointMatrix;

static PointCloud& MATRIX_TO_POINTCLOUD(const PointMatrix& point_matrix)
{
	PointCloud point_cloud;
	size_t point_size = point_matrix.rows();
	
	for (size_t i = 0; i < point_size; i ++)
	{
		point_cloud.push_back(Point(point_matrix(i,0), point_matrix(i,1), point_matrix(i,2)));
	}

	return point_cloud;
}

static PointMatrix& POINTCLOUD_TO_MATRIX(const PointCloud& point_cloud)
{
	PointMatrix point_matrix;
	size_t point_size = point_cloud.size();
	point_matrix.resize(point_size, 3);

	for (size_t i = 0; i < point_size; i ++)
	{
		const Point& point = point_cloud.at(i);

		point_matrix(i, 0) = point.x;
		point_matrix(i, 1) = point.y;
		point_matrix(i, 2) = point.z;

	}

	return point_matrix;
}

#endif