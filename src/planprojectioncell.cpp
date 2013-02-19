
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "planprojectioncell.h"
#include "plancloud.h"

PlanProjectionCell::PlanProjectionCell()
{
	proj.setModelType (pcl::SACMODEL_PLANE);
}

planCloudsPtr_t PlanProjectionCell::compute(planCloudsPtr_t planCloudListPtr)
{
	for(planClouds_t::size_type k=0; k<planCloudListPtr->size(); k++)
	{
		plan_cloud_ptr_ = boost::make_shared<PlanCloud > (planCloudListPtr->at(k));
		proj.setInputCloud (plan_cloud_ptr_->cloud());
		proj.setModelCoefficients (plan_cloud_ptr_->coefficients());
		proj.filter (*(plan_cloud_ptr_->cloud()));
	}
	return planCloudListPtr;
}
