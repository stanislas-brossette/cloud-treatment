#include <boost/make_shared.hpp>

#include "xyzswitchcell.h"

#include "plancloud.h"

XYZSwitchCell::XYZSwitchCell():
	Cell()
{
	parameters()["name"] = "XYZSwitchCell";
	cloud_ptr_ = boost::make_shared<pointCloud_t >();
}


planCloudsPtr_t XYZSwitchCell::compute(planCloudsPtr_t planCloudListPtr)
{
	cell_name_ = boost::get<std::string>(parameters()["name"]);

	for(planClouds_t::size_type j=0; j<planCloudListPtr->size(); ++j)
	{
		float x, y, z;
		cloud_ptr_ = planCloudListPtr->at(j).cloud();
		for(planClouds_t::size_type i = 0;
			i<cloud_ptr_->points.size (); ++i)
		{
			if(!isnan(cloud_ptr_->points[i].x))
			{
				x = cloud_ptr_->points[i].z;
				y = -cloud_ptr_->points[i].x;
				z = -cloud_ptr_->points[i].y;

				cloud_ptr_->points[i].x = x;
				cloud_ptr_->points[i].y = y;
				cloud_ptr_->points[i].z = z;
			}
		}
	}
	return planCloudListPtr;
}
