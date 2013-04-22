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
	parameters()["min_"] = 0;
	parameters()["max_"] = 2.7;
}


planCloudsPtr_t PassThroughFilterCell::compute(planCloudsPtr_t planCloudListPtr)
{
	min_ = static_cast<float>(parameters()["min_"]);
	max_ = static_cast<float>(parameters()["max_"]);

	for(pointCloudPoints_t::size_type j = 0;
		j<planCloudListPtr->size(); ++j)
	{
		cloud_ptr_ = planCloudListPtr->at(j).cloud();
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud (cloud_ptr_);
		pass.setFilterFieldName (axis);
		pass.setFilterLimits (min_, max_);
		//pass.setFilterLimitsNegative (true);
		pass.filter (*cloud_ptr_);
	}
	return planCloudListPtr;
}
