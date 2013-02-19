#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <boost/make_shared.hpp>
#include <pcl/filters/voxel_grid.h>

#include "filtercell.h"
#include "plancloud.h"

FilterCell::FilterCell()
{
	cloud_ptr_ = boost::make_shared<pointCloud_t>();
	cloud2_ptr_ = boost::make_shared<pointCloud2_t>();
	leafX_ = 0.01f;
	leafY_ = 0.01f;
	leafZ_ = 0.01f;
}

FilterCell::FilterCell(float leafX,float  leafY,float leafZ)
{
	cloud_ptr_ = boost::make_shared<pointCloud_t>();
	cloud2_ptr_ = boost::make_shared<pointCloud2_t>();
	leafX_ = leafX;
	leafY_ = leafY;
	leafZ_ = leafZ;
}

planCloudsPtr_t FilterCell::compute(planCloudsPtr_t planCloudListPtr)
{
	for(pointCloudPoints_t::size_type j = 0;
		j<planCloudListPtr->size(); j++)
	{
		cloud_ptr_ = planCloudListPtr->at(j).cloud();
		toROSMsg (*cloud_ptr_, *cloud2_ptr_);
		pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
		sor.setInputCloud (cloud2_ptr_);
		sor.setLeafSize (leafX_, leafY_, leafZ_);
		sor.filter (*(planCloudListPtr->at(j).cloud2()));
		pcl::fromROSMsg (*(planCloudListPtr->at(j).cloud2()), *cloud_ptr_);
	}
	return planCloudListPtr;
}
