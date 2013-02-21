#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <boost/optional.hpp>

#include <boost/make_shared.hpp>
#include <sensor_msgs/PointCloud2.h>

#include "plancloud.h"

PlanCloud::PlanCloud()
{
	cloud_ = boost::make_shared<pointCloud_t >();
	coefficients_ = boost::make_shared<pcl::ModelCoefficients >();
	cloud2_ = boost::make_shared<pointCloud2_t >();
}

void PlanCloud::reset()
{
	this->cloud_->clear();
}

std::ostream& PlanCloud::print(std::ostream& o) const throw ()
{
	o <<"PointCloud's size: "
	 << this->cloud_->points.size () << std::endl;
	if(coefficients_->values.size() != 0)
	{
		o << "Model coefficients: " << coefficients_->values[0] << " "
		  << coefficients_->values[1] << " "
		  << coefficients_->values[2] << " "
		  << coefficients_->values[3] << std::endl;
	}

	if(origin_)
	{
		o << "Origin = \n\t[" << (*origin_).transpose() << "]" << std::endl;
	}

	if(frame_)
	{
		o << "Frame = \n "
		<< "\tT = [" << frame_->at(0).transpose() << " ]\n"
		<< "\tB = [" << frame_->at(1).transpose() << " ]\n"
		<< "\tN = [" << frame_->at(2).transpose() << " ]\n";
	}
	return o;
}

void PlanCloud::find_origin()
{
	origin_ = Eigen::Vector3d();
	//The origin is considered to be the average of all the points
	for(pointCloudPoints_t::size_type i = 0; i < cloud_->points.size(); ++i)
	{
		origin_->x() += cloud_->points[i].x;
		origin_->y() += cloud_->points[i].y;
		origin_->z() += cloud_->points[i].z;
	}
	origin_->x() /= static_cast<float>(cloud_->points.size());
	origin_->y() /= static_cast<float>(cloud_->points.size());
	origin_->z() /= static_cast<float>(cloud_->points.size());
}

void PlanCloud::find_frame()
{
	frame_ = std::vector<Eigen::Vector3d>(3, Eigen::Vector3d());

	float a = coefficients_->values[0];
	float b = coefficients_->values[1];
	float c = coefficients_->values[2];

	//frame_[2] is the normal vector to the planCloud
	frame_->at(2) = Eigen::Vector3d(a, b, c);

	if(a!=0 || b!=0)
	{
		frame_->at(1) = Eigen::Vector3d(-b, a, 0.0);
		frame_->at(0) = Eigen::Vector3d(a*c, b*c, -b*b-a*a);
	}
	else
	{
		frame_->at(2) = Eigen::Vector3d(0.0, 0.0, 1.0);
		frame_->at(1) = Eigen::Vector3d(0.0, 1.0, 0.0);
		frame_->at(0) = Eigen::Vector3d(1.0, 0.0, 0.0);
	}

	for (int i = 0; i<3; i++)
	{
		frame_->at(i).normalize();
	}
}

Eigen::Vector3d PlanCloud::project_point(pointCloudPoints_t::size_type i)
{
	Eigen::Vector3d projectedPoint = Eigen::Vector3d();
	Eigen::Vector3d point = Eigen::Vector3d(cloud()->points[i].x, cloud()->points[i].y, cloud()->points[i].z);
	projectedPoint.x() = (point - *origin_).dot(frame_->at(0));
	projectedPoint.y() = (point - *origin_).dot(frame_->at(1));
	projectedPoint.z() = (point - *origin_).dot(frame_->at(2));

	return projectedPoint;
}


