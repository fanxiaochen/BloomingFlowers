#ifndef TYPES_WRAPPER_H
#define TYPES_WRAPPER_H

#include <QMutex>

#include <base/matrix.hpp>

#include <Deform.h>

#include "point_cloud.h"
#include "mesh_model.h"
#include "flower.h"

typedef cpd::MatrixType<float, 3>::MatrixD PointMatrix;
typedef cpd::MatrixType<float, 3>::Matrix CorresMatrix;

typedef Deform::VectorF VectorFArray;
typedef Deform::VectorI VectorIArray;

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

static PointMatrix FLOWER_TO_MATRIX(Flower& flower)
{
    PointMatrix point_matrix;

    Petals& petals = flower.getPetals();

    size_t point_size = 0;
    for (size_t i = 0, i_end = petals.size(); i < i_end; ++ i)
    {
        Petal& petal = petals[i];
        std::vector<int> hard_idx = petal.getHardCtrsIndex();
        point_size += hard_idx.size();
    }
    
    point_matrix.resize(point_size, 3);

    int k = 0;
    for (size_t i = 0, i_end = petals.size(); i < i_end; ++ i)
    {
        Petal& petal = petals[i];
        std::vector<int> hard_idx = petal.getHardCtrsIndex();
        osg::ref_ptr<osg::Vec3Array> vertices = petal.getVertices();

        for (size_t j = 0, j_end = hard_idx.size(); j < j_end; ++ j)
        {
            const osg::Vec3& point = vertices->at(hard_idx[j]);

            point_matrix(k, 0) = point.x();
            point_matrix(k, 1) = point.y();
            point_matrix(k, 2) = point.z();

            k++;
        }
    }

    return point_matrix;
}

static void MATRIX_TO_FLOWER(const PointMatrix& point_matrix, Flower& flower)
{
    size_t point_size = point_matrix.rows();

    Petals& petals = flower.getPetals();

    int k = 0;
    for (size_t i = 0, i_end = petals.size(); i < i_end; ++ i)
    {
        Petal& petal = petals[i];
        osg::ref_ptr<osg::Vec3Array>& hard_ctrs = petal.getHardCtrs();
        std::vector<int> hard_idx = petal.getHardCtrsIndex();
        hard_ctrs->clear();

        for (size_t j = 0, j_end = hard_idx.size(); j < j_end; ++ j)
        {
            osg::Vec3 point;
            point.x() = point_matrix(k,0);
            point.y() = point_matrix(k,1);
            point.z() = point_matrix(k,2);

            petal.getHardCtrs()->push_back(point);

            k++;
        }
    }

}

static void MESHMODEL_TO_VECTORARRAY(MeshModel& mesh_model, VectorFArray& vectorf_array, VectorIArray& vectori_array)
{
    vectori_array.clear();
    vectorf_array.clear();

    osg::ref_ptr<osg::Vec3Array> points = mesh_model.getVertices();
    size_t point_size = points->size();

    for (size_t i = 0; i < point_size; i ++)
    {
        const osg::Vec3& point = points->at(i);

        vectorf_array.push_back(point.x());
        vectorf_array.push_back(point.y());
        vectorf_array.push_back(point.z());

        vectori_array.push_back(i);
    }
}

static void POINTCLOUD_TO_VECTORARRAY(PointCloud& point_cloud, VectorFArray& vectorf_array, VectorIArray& vectori_array)
{
    vectori_array.clear();
    vectorf_array.clear();

    size_t point_size = point_cloud.size();

    for (size_t i = 0; i < point_size; i ++)
    {
        const Point& point = point_cloud.at(i);

        vectorf_array.push_back(point.x);
        vectorf_array.push_back(point.y);
        vectorf_array.push_back(point.z);

        vectori_array.push_back(i);
    }
}


#endif