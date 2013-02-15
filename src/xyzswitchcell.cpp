#include <boost/make_shared.hpp>

#include "xyzswitchcell.h"

#include "plancloud.h"

XYZSwitchCell::XYZSwitchCell()
{
	cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
}


boost::shared_ptr<std::vector<PlanCloud> > XYZSwitchCell::compute(boost::shared_ptr<std::vector<PlanCloud> > planCloudListPtr)
{
	for(int j=0; j<planCloudListPtr->size(); j++)
	{
		float x, y, z;
		cloud_ptr_ = planCloudListPtr->at(j).cloud();
		for(int i = 0; i<cloud_ptr_->points.size (); i++)
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
