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
	cloud_ptr_ = boost::make_shared<pointCloud_t>();
	leafX_ = 0.01f;
	leafY_ = 0.01f;
	leafZ_ = 0.01f;
}

FilterCell::FilterCell(float leafX,float  leafY,float leafZ)
{
	cloud_ptr_ = boost::make_shared<pointCloud_t>();
	leafX_ = leafX;
	leafY_ = leafY;
	leafZ_ = leafZ;
}

planCloudsPtr_t FilterCell::compute(planCloudsPtr_t planCloudListPtr)
{
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

planCloudsPtr_t FilterCell::computePassThrough(planCloudsPtr_t planCloudListPtr, float maxDistance)
{
	for(pointCloudPoints_t::size_type j = 0;
		j<planCloudListPtr->size(); ++j)
	{
		cloud_ptr_ = planCloudListPtr->at(j).cloud();
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud (cloud_ptr_);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.0, maxDistance);
		//pass.setFilterLimitsNegative (true);
		pass.filter (*cloud_ptr_);
	}
	return planCloudListPtr;
}

planCloudsPtr_t FilterCell::computePassThrough(planCloudsPtr_t planCloudListPtr, std::string axis, float min, float max)
{
	for(pointCloudPoints_t::size_type j = 0;
		j<planCloudListPtr->size(); ++j)
	{
		cloud_ptr_ = planCloudListPtr->at(j).cloud();
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud (cloud_ptr_);
		pass.setFilterFieldName (axis);
		pass.setFilterLimits (min, max);
		//pass.setFilterLimitsNegative (true);
		pass.filter (*cloud_ptr_);
	}
	return planCloudListPtr;
}
