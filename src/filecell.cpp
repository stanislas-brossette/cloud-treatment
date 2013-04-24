#include <vector>
#include <string>
#include <iostream>
#include <stdexcept>
#include <cassert>

#include <boost/shared_ptr.hpp>
#include <pcl/io/pcd_io.h>

#include "filecell.h"


FileCell::FileCell()
{
	cell_name() = "FileCell";
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
//	std::cout << "Read file " << path << std::endl;
//	std::cout << "This cloud has " << planCloudTest.cloud()->points.size () << " points." << std::endl;
	assert(planCloudTest.cloud()->points.size () != 0
			&& "Problem encountered while reading the pcl file./n Cloud empty.");
	if(planCloudTest.cloud()->points.size () == 0)
	{
		throw std::runtime_error("Problem encountered while reading the pcl file./n"
								 "Cloud empty.");
	}
	planCloudListPtr->push_back(planCloudTest);
	return planCloudListPtr;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr FileCell::sync_color(std::string path)
{
	path = "../datafiles/" + path + ".pcd";
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (path, *cloud) == -1) //* load the file
	  {
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	  }
	  std::cout << "Loaded "
				<< cloud->width * cloud->height
				<< " data points from test_pcd.pcd with the following fields: "
				<< std::endl;

	return cloud;
}
