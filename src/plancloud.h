#ifndef PLANCLOUD_H
#define PLANCLOUD_H

#include <iostream>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>


class PlanCloud
{
public:
	PlanCloud();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud()
	{
		return cloud_;
	}
	const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud() const
	{
		return cloud_;
	}

	pcl::ModelCoefficients::Ptr coefficients()
	{
		return coefficients_;
	}
	const pcl::ModelCoefficients::Ptr coefficients() const
	{
		return coefficients_;
	}

	int size()
	{
		return cloud_->points.size();
	}

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
	pcl::ModelCoefficients::Ptr coefficients_;
};

#endif // PLANCLOUD_H
