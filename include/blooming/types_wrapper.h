#ifndef TYPES_WRAPPER_H
#define TYPES_WRAPPER_H

#include <QMutex>

#include <base/matrix.hpp>

#include "point_cloud.h"

typedef double value_type;
typedef cpd::MatrixType<value_type, 3>::MatrixD PointMatrix;

static void MATRIX_TO_POINTCLOUD(const PointMatrix& point_matrix, PointCloud& point_cloud)
{
	QWriteLocker locker(&point_cloud.getReadWriteLock());

	size_t point_size = point_matrix.rows();
	
	for (size_t i = 0; i < point_size; i ++)
	{
		Point& point = point_cloud.at(i);
		point.x = point_matrix(i,0);
		point.y = point_matrix(i,1);
		point.z = point_matrix(i,2);
		//point_cloud.push_back(Point(point_matrix(i,0), point_matrix(i,1), point_matrix(i,2)));
	}
}
static PointMatrix POINTCLOUD_TO_MATRIX(const PointCloud& point_cloud)
{
	PointMatrix point_matrix;
	size_t point_size = point_cloud.size();
	point_matrix.resize(point_size, 3);

	for (size_t i = 0; i < point_size; i ++)
	{
		const Point& point = point_cloud.at(i);

		point_matrix(i, 0) = value_type(point.x);
		point_matrix(i, 1) = value_type(point.y);
		point_matrix(i, 2) = value_type(point.z);

	}

	return point_matrix;
}

#endif