#include <iostream>

#include <boost/make_shared.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sensor_msgs/PointCloud2.h>

#include "filtercell.h"
#include "plancloud.h"

FilterCell::FilterCell():
	Cell("FilterCell")
{
	cloud_ptr_ = boost::make_shared<pointCloud_t>();
	parameters()["leafX"] = 0.01;
	parameters()["leafY"] = 0.01;
	parameters()["leafZ"] = 0.01;
}

planCloudsPtr_t FilterCell::compute(planCloudsPtr_t planCloudListPtr)
{
	leafX_ = static_cast<float>(parameters()["leafX"]);
	leafY_ = static_cast<float>(parameters()["leafY"]);
	leafZ_ = static_cast<float>(parameters()["leafZ"]);

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
