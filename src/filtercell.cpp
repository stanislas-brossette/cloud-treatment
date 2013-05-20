#include <iostream>

#include <boost/make_shared.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>

#include "filtercell.h"
#include "plancloud.h"

FilterCell::FilterCell():
	Cell()
{
	parameters()["name"] = "FilterCell";
	cloud_ptr_ = boost::make_shared<pointCloud_t>();
	parameters()["leafX"] = 0.01;
	parameters()["leafY"] = 0.01;
	parameters()["leafZ"] = 0.01;
	parameters()["leafSize"] = 0;
}

planCloudsPtr_t FilterCell::compute(planCloudsPtr_t planCloudListPtr)
{
	cell_name_ = boost::get<std::string>(parameters()["name"]);

	leafX_ = static_cast<float>(boost::get<double>(parameters()["leafX"]));
	leafY_ = static_cast<float>(boost::get<double>(parameters()["leafY"]));
	leafZ_ = static_cast<float>(boost::get<double>(parameters()["leafZ"]));
	leafSize_ = static_cast<float>(boost::get<double>(parameters()["leafSize"]));

	if (leafSize_ != 0)
	{
		leafX_ = leafSize_;
		leafY_ = leafSize_;
		leafZ_ = leafSize_;
	}

	for(pointCloudPoints_t::size_type j = 0;
		j<planCloudListPtr->size(); ++j)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		cloud_ptr_ = planCloudListPtr->at(j).cloud();
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud (cloud_ptr_);
		sor.setLeafSize (leafX_, leafY_, leafZ_);
		sor.filter (*cloud_filtered);
		*cloud_ptr_  = *cloud_filtered;
	}
	return planCloudListPtr;
}
