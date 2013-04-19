#include <iostream>

#include <boost/make_shared.hpp>
#include <pcl/filters/passthrough.h>

#include "passthroughfiltercell.h"
#include "plancloud.h"

PassThroughFilterCell::PassThroughFilterCell()
{
	cell_name() = "PassThroughFilterCell";
	cloud_ptr_ = boost::make_shared<pointCloud_t>();
	axis = "z";
	parameters()["min"] = 0;
	parameters()["max"] = 2.7;
}


planCloudsPtr_t PassThroughFilterCell::compute(planCloudsPtr_t planCloudListPtr)
{
	min = static_cast<float>(parameters()["min"]);
	max = static_cast<float>(parameters()["max"]);

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