#ifndef TYPES_WRAPPER_H
#define TYPES_WRAPPER_H

#include <QMutex>

#include <base/matrix.hpp>

#include "point_cloud.h"
#include "mesh_model.h"

typedef cpd::MatrixType<float, 3>::MatrixD PointMatrix;
typedef cpd::MatrixType<float, 3>::Matrix CorresMatrix;

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

		point_matrix(i, 0) = float(point.x);
		point_matrix(i, 1) = float(point.y);
		point_matrix(i, 2) = float(point.z);

	}

	return point_matrix;
}

static void MATRIX_TO_MESHMODEL(const PointMatrix& point_matrix, MeshModel& mesh_model)
{
	QWriteLocker locker(&mesh_model.getReadWriteLock());

	size_t point_size = point_matrix.rows();

	osg::ref_ptr<osg::Vec3Array> vertices = mesh_model.getVertices();

	for (size_t i = 0; i < point_size; i ++)
	{
		osg::Vec3& point = vertices->at(i);
		point.x() = point_matrix(i,0);
		point.y() = point_matrix(i,1);
		point.z() = point_matrix(i,2);
	}
}

static PointMatrix MESHMODEL_TO_MATRIX(MeshModel& mesh_model)
{
	PointMatrix point_matrix;
	osg::ref_ptr<osg::Vec3Array> points = mesh_model.getVertices();
	size_t point_size = points->size();
	point_matrix.resize(point_size, 3);

	for (size_t i = 0; i < point_size; i ++)
	{
		const osg::Vec3& point = points->at(i);

		point_matrix(i, 0) = float(point.x());
		point_matrix(i, 1) = float(point.y());
		point_matrix(i, 2) = float(point.z());

	}

	return point_matrix;
}

#endif