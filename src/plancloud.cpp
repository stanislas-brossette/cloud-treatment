#include <iostream>
#include <vector>

#include <boost/make_shared.hpp>
#include <boost/optional.hpp>
#include <eigen3/Eigen/Dense>

#include "plancloud.h"
#include "typedefs.h"

PlanCloud::PlanCloud()
	: cloud_ (boost::make_shared<pointCloud_t >()),
	  keyPoints_ (boost::make_shared<pointCloud_t >()),
	  normals_ (boost::make_shared<normalCloud_t >()),
	  descriptors_ (boost::make_shared<descriptorCloud_t >()),
	  coefficients_ (boost::make_shared<pcl::ModelCoefficients >())
{
}

PlanCloud::PlanCloud(const PlanCloud& planCloud)
	: cloud_ (boost::make_shared<pointCloud_t >()),
	  keyPoints_ (boost::make_shared<pointCloud_t >()),
	  normals_(planCloud.normals_),
	  descriptors_ (planCloud.descriptors_),
	  coefficients_ (boost::make_shared<pcl::ModelCoefficients >()),
	  origin_(planCloud.origin_),
	  T_(planCloud.T_),
	  B_(planCloud.B_),
	  N_(planCloud.N_),
	  frame_(planCloud.frame_)
{
	*cloud_ = *(planCloud.cloud_);
	*keyPoints_ = *(planCloud.keyPoints_);
	*coefficients_ = *(planCloud.coefficients_);
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
		  << "\tT = [" << T_.transpose() << " ]\n"
		  << "\tB = [" << B_.transpose() << " ]\n"
		  << "\tN = [" << N_.transpose() << " ]\n";
	}
	return o;
}

std::ostream& PlanCloud::print(Verbose v, std::ostream& o) const throw ()
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
		  << "\tT = [" << T_.transpose() << " ]\n"
		  << "\tB = [" << B_.transpose() << " ]\n"
		  << "\tN = [" << N_.transpose() << " ]\n";
	}

	if(v.level()>0)
	{
		o << "Point List (in frame and origin referential): "<<std::endl;
		for (pointCloudPoints_t::size_type i = 0; i < cloud_->points.size(); ++i)
		{
			o << cloud_->points[i] << std::endl;
		}
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
	frame_ = Eigen::Matrix3d();
	float a = coefficients_->values[0];
	float b = coefficients_->values[1];
	float c = coefficients_->values[2];
	N_ << a, b, c;
	if(a!=0 || b!=0)
	{
		B_ << -b, a, 0.0;
		T_ << a*c, b*c, -b*b-a*a;
	}
	else
	{
		N_ << 0.0, 0.0, 1.0;
		B_ << 0.0, 1.0, 0.0;
		T_ << 1.0, 0.0, 0.0;
	}
	T_.normalize();
	B_.normalize();
	N_.normalize();

	//Here we ensure that N.origin<0 so that all the normals point toward the camera
	if(N_.dot(*origin_)>0)
	{
		T_ = -T_;
		N_ = -N_;
	}
	*frame_ << T_, B_, N_;
}

void PlanCloud::project_points_on_frame()
{
	for (pointCloudSize_t i = 0; i < cloud_->points.size(); ++i)
	{
		Eigen::Vector3d pointI = project_point(i);
		cloud_->points[i].x = static_cast<float>(pointI.x());
		cloud_->points[i].y = static_cast<float>(pointI.y());
		cloud_->points[i].z = static_cast<float>(pointI.z());
	}
}

void PlanCloud::set_trigo_order()
{
	if(cloud_->points.size() < 3)
		throw std::runtime_error("This is not a correct hull convex");

	Eigen::Vector3d edge1;
	Eigen::Vector3d edge2;
	edge1 << cloud_->points[1].x-cloud_->points[0].x, cloud_->points[1].y-cloud_->points[0].y, cloud_->points[1].z-cloud_->points[0].z;
	edge2 << cloud_->points[2].x-cloud_->points[1].x, cloud_->points[2].y-cloud_->points[1].y, cloud_->points[2].z-cloud_->points[1].z;

	//	std::cout<<"edge1 = "<<edge1.transpose() << std::endl;
	//	std::cout<<"edge2 = "<<edge2.transpose() << std::endl;
	//	std::cout<<"N_ = "<<N_.transpose() << std::endl;

	if ((edge1.cross(edge2)).dot(N_)<0)
	{
		std::cout<<"inverting the points"<<std::endl;
		pointCloudPtr_t cloudInverted = boost::make_shared<pointCloud_t >();
		pointCloudPoints_t::size_type cloudSize = cloud_->points.size();
		cloudInverted->reserve(cloudSize);
		for (pointCloudPoints_t::size_type i = 0; i < cloudSize; ++i)
		{
			cloudInverted->push_back(cloud_->points[cloudSize-i-1]);
		}
		cloud_ = cloudInverted;
	}
	else if ( (edge1.cross(edge2)).dot(N_)==0)
	{
		std::cout << "points are aligned, we have a problem in PlanCloud::set_trigo_order()" << std::endl;
	}
	else
	{
	}
}

Eigen::Vector3d PlanCloud::project_point(pointCloudPoints_t::size_type i) const
{
	Eigen::Vector3d projectedPoint = Eigen::Vector3d();
	Eigen::Vector3d point = Eigen::Vector3d(cloud()->points[i].x, cloud()->points[i].y, cloud()->points[i].z);
	projectedPoint.x() = (point - *origin_).dot(T_);
	projectedPoint.y() = (point - *origin_).dot(B_);
	projectedPoint.z() = (point - *origin_).dot(N_);
	return projectedPoint;
}


