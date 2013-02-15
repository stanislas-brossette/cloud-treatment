#ifndef PLANCLOUD_H
#define PLANCLOUD_H

#include <iostream>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <sensor_msgs/PointCloud2.h>


class PlanCloud
{
public:
	PlanCloud();
	void display();
	void info();

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

	sensor_msgs::PointCloud2::Ptr cloud2()
	{
		return cloud2_;
	}
	const sensor_msgs::PointCloud2::Ptr cloud2() const
	{
		return cloud2_;
	}

	int size()
	{
		return cloud_->points.size();
	}



private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
	sensor_msgs::PointCloud2::Ptr cloud2_;
	pcl::ModelCoefficients::Ptr coefficients_;
};

#endif // PLANCLOUD_H
