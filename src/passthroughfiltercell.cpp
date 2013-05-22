#include <iostream>

#include <boost/make_shared.hpp>

#include <pcl/filters/passthrough.h>

#include "passthroughfiltercell.h"
#include "plancloud.h"
#include "typedefs.h"

PassThroughFilterCell::PassThroughFilterCell():
	Cell()
{
	parameters()["name"] = "PassThroughFilterCell";
	cloud_ptr_ = boost::make_shared<pointCloud_t>();
	parameters()["axis"] = "z";
	parameters()["min"] = 0;
	parameters()["max"] = 2.7;
	parameters()["keep_organized"] = 0;
}

planCloudsPtr_t PassThroughFilterCell::compute(planCloudsPtr_t planCloudListPtr)
{
	cell_name_ = boost::get<std::string>(parameters()["name"]);

	axis_ = boost::get<std::string>(parameters()["axis"]);
	min_ = static_cast<float>(boost::get<double>(parameters()["min"]));
	max_ = static_cast<float>(boost::get<double>(parameters()["max"]));
	keep_organized_ = static_cast<int>(boost::get<double>(parameters()["keep_organized"]));

	for(pointCloudPoints_t::size_type j = 0;
		j<planCloudListPtr->size(); ++j)
	{
		cloud_ptr_ = planCloudListPtr->at(j).cloud();
		pcl::PassThrough<pointT> pass;
		if(keep_organized_)
			pass.setKeepOrganized (true);
		pass.setInputCloud (cloud_ptr_);
		pass.setFilterFieldName (axis_);
		pass.setFilterLimits (min_, max_);
		//pass.setFilterLimitsNegative (true);
		pass.filter (*cloud_ptr_);
	}
	return planCloudListPtr;
}
