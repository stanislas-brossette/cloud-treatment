#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <boost/make_shared.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>

#include "filtercell.h"
#include "plancloud.h"

FilterCell::FilterCell()
{
	cell_name() = "FilterCell";
	cloud_ptr_ = boost::make_shared<pointCloud_t>();
	parameters()["leafX_"] = 0.01;
	parameters()["leafY_"] = 0.01;
	parameters()["leafZ_"] = 0.01;
}

planCloudsPtr_t FilterCell::compute(planCloudsPtr_t planCloudListPtr)
{
	leafX_ = static_cast<float>(parameters()["leafX_"]);
	leafY_ = static_cast<float>(parameters()["leafY_"]);
	leafZ_ = static_cast<float>(parameters()["leafZ_"]);

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
