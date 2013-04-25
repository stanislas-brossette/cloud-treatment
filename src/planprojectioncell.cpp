#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include "plancloud.h"
#include "planprojectioncell.h"

PlanProjectionCell::PlanProjectionCell():
	Cell("PlanProjectionCell")
{
	proj.setModelType (pcl::SACMODEL_PLANE);
}

planCloudsPtr_t PlanProjectionCell::compute(planCloudsPtr_t planCloudListPtr)
{
	for(planClouds_t::size_type k=0; k<planCloudListPtr->size(); k++)
	{
		proj.setInputCloud (planCloudListPtr->at(k).cloud());
		proj.setModelCoefficients (planCloudListPtr->at(k).coefficients());
		proj.filter (*(planCloudListPtr->at(k).cloud()));
	}
	return planCloudListPtr;
}
