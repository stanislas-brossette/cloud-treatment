#include <boost/make_shared.hpp>
#include <pcl/visualization/cloud_viewer.h>

# include "hullconvexcell.h"
# include "typedefs.h"

HullConvexCell::HullConvexCell():
	Cell()
{
	parameters()["name"] = "HullConvexCell";
	point_cloud_ptr_ = boost::make_shared<pointCloud_t > ();
}

planCloudsPtr_t HullConvexCell::compute(planCloudsPtr_t planCloudListPtr)
{
	cell_name_ = boost::get<std::string>(parameters()["name"]);

	for(planClouds_t::size_type k=0; k<planCloudListPtr->size(); k++)
	{
		point_cloud_ptr_ = boost::make_shared<pointCloud_t > ();
		chull.setInputCloud (planCloudListPtr->at(k).cloud());
		chull.setDimension(2);
		chull.reconstruct (*point_cloud_ptr_);
		planCloudListPtr->at(k).cloud() = point_cloud_ptr_;
	}

	return planCloudListPtr;
}

