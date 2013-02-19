#include <vector>
#include <string>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <pcl/io/pcd_io.h>
#include <stdexcept>
#include <cassert>

#include "filecell.h"


FileCell::FileCell()
{
}

planCloudsPtr_t FileCell::compute(planCloudsPtr_t planCloudListPtr)
{
	return planCloudListPtr;
}

planCloudsPtr_t FileCell::sync(std::string path, planCloudsPtr_t planCloudListPtr)
{
	PlanCloud planCloudTest = PlanCloud();
	pcl::PCDReader reader;
	reader.read(path, *(planCloudTest.cloud()));
	assert(planCloudTest.cloud()->points.size () != 0
			&& "Problem encountered while reading the pcl file.\n Cloud empty.");
	if(planCloudTest.cloud()->points.size () == 0)
	{
		throw std::runtime_error("Problem encountered while reading the pcl file.\n"
								 "Cloud empty.");
	}
	planCloudListPtr->push_back(planCloudTest);
	return planCloudListPtr;
}
