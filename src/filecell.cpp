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

boost::shared_ptr<std::vector<PlanCloud> > FileCell::compute(boost::shared_ptr<std::vector<PlanCloud> >)
{
}

boost::shared_ptr<std::vector<PlanCloud> > FileCell::sync(std::string path, boost::shared_ptr<std::vector<PlanCloud> > planCloudListPtr)
{
	PlanCloud planCloudTest = PlanCloud();
	pcl::PCDReader reader;
	reader.read(path, *(planCloudTest.cloud()));
	assert(planCloudTest.cloud()->points.size () != 0 && "Problem encountered while reading the pcl file.\n Cloud empty.");
	if(planCloudTest.cloud()->points.size () == 0)
	{
		throw std::runtime_error("Problem encountered while reading the pcl file.\n Cloud empty.");
	}
	planCloudListPtr->push_back(planCloudTest);
	return planCloudListPtr;
}
