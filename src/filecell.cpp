#include <cassert>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <pcl/io/pcd_io.h>

#include "filecell.h"

FileCell::FileCell():
	Cell()
{
	parameters()["name"] = "FileCell";
}

planCloudsPtr_t FileCell::compute(planCloudsPtr_t planCloudListPtr)
{
	cell_name_ = boost::get<std::string>(parameters()["name"]);
	return planCloudListPtr;
}

planCloudsPtr_t FileCell::sync(std::string path, planCloudsPtr_t planCloudListPtr)
{
	PlanCloud planCloudTest = PlanCloud();
	pcl::PCDReader reader;
	reader.read(path, *(planCloudTest.cloud()));
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
