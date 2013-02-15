#include <iostream>
#include <boost/make_shared.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include "plancloud.h"

PlanCloud::PlanCloud()
{
	cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	coefficients_ = boost::make_shared<pcl::ModelCoefficients >();
}

void PlanCloud::display()
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
