#include "fitting_error_measurer.h"
#include "flower.h"
#include "point_cloud.h"
#include "mesh_model.h"





void FittingErrorMeasurer::setFlowerAndPointCloud(Flower* ptr_flower, PointCloud* ptr_cloud)
{
	ptr_flower_ = ptr_flower;
	ptr_cloud_ = ptr_cloud;
	point_tri_pairs_.clear();
	err_ = 0;
}


void FittingErrorMeasurer::computeDisFromCloud2Flower()
{
	size_t num_p = ptr_cloud_->points.size();
	point_tri_pairs_.resize(num_p);
	for (size_t i = 0; i != num_p; ++i)
	{
		comptueNearestTriangle(i);
	}
}

double FittingErrorMeasurer::getMeanDis()
{
	double sum_dis = 0;
	size_t num_p = point_tri_pairs_.size();
	if (num_p == 0)
	{
		std::cout << "Run [FittingErrorMeasurer]computeDisFromCloud2Flower first" << std::endl;
		return 0;
	}
	for (auto& ptp : point_tri_pairs_)
	{
		sum_dis += std::sqrt(ptp.dis2_);
	}
	err_ = sum_dis / num_p;
	return err_;
}


void FittingErrorMeasurer::comptueNearestTriangle(int point_id)
{
	Point& p_pcl = ptr_cloud_->points.at(point_id);
	osg::Vec3 p_osg(p_pcl.x, p_pcl.y, p_pcl.z);

	std::map<double, PointTriPair> ptps;
	for (size_t i = 0; i != ptr_flower_->getPetals().size(); ++i)
	{
		PointTriPair ptp = comptueNearestTriangle(p_osg, i);
		ptps[ptp.dis2_] = ptp;
	}
	
	auto& mit = ptps.begin();
	mit->second.point_id_ = point_id;
	point_tri_pairs_[point_id] = mit->second;
}


PointTriPair FittingErrorMeasurer::comptueNearestTriangle(osg::Vec3 p, int petal_id)
{
	typedef  std::vector<int> TriFacet;

	Petal& petal = ptr_flower_->getPetals().at(petal_id);
	std::vector< TriFacet >& triangles = petal.getFaces();

	std::vector<double> dis2s( triangles.size(), 0);
	osg::Vec3 proj_pos; 
	double dis2;
	for (size_t i = 0; i != triangles.size(); ++i)
	{
		dis2 = petal.disancePoint3Tri3(p, i, proj_pos);
		dis2s[i] = dis2;
	}

	int tri_id = std::min_element(dis2s.begin(), dis2s.end()) - dis2s.begin();
	dis2 = petal.disancePoint3Tri3(p, tri_id, proj_pos);

	PointTriPair ptp;
	ptp.p_ = p;
	ptp.petel_id_ = petal_id;
	ptp.tri_id_ = tri_id;
	ptp.closest_p_ = proj_pos;
	ptp.dis2_ = dis2;

	return ptp;
}


