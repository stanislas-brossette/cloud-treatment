#include <boost/make_shared.hpp>
#include "plancloud.h"

PlanCloud::PlanCloud()
{
	cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
//	cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	coefficients_ = boost::make_shared<pcl::ModelCoefficients >();
}
