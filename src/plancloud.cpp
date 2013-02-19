#include <iostream>

#include <boost/make_shared.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>

#include "plancloud.h"

PlanCloud::PlanCloud()
{
	cloud_ = boost::make_shared<pointCloud_t >();
	coefficients_ = boost::make_shared<pcl::ModelCoefficients >();
	cloud2_ = boost::make_shared<pointCloud2_t >();
}

void PlanCloud::display_cloud()
{
	pcl::visualization::CloudViewer viewer("SimpleCloudViewer");
	viewer.showCloud(this->cloud_);
	while(!viewer.wasStopped())
	{
	}
}

void PlanCloud::info()
{
	std::cout<<"This PointCloud has: "
			<< this->cloud_->points.size () << " data points." << std::endl;

}

void PlanCloud::reset()
{
	this->cloud_->clear();
}

void PlanCloud::display_planar_components()
{
	std::cout << "Model coefficients: " << coefficients_->values[0] << " "
			  << coefficients_->values[1] << " "
			  << coefficients_->values[2] << " "
			  << coefficients_->values[3] << std::endl
			  << "Cloud made of " << cloud_->points.size()
			  << " data points."<<std::endl<<std::endl;
}

void PlanCloud::update_cloud2_from_cloud()
{
	toROSMsg (*cloud_, *cloud2_);
}

void PlanCloud::update_cloud_from_cloud2()
{
	pcl::fromROSMsg (*cloud2_, *cloud_);
}
