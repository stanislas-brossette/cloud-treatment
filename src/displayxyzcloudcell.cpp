#include "application.h"
#include "displayxyzcloudcell.h"
#include "plancloud.h"

DisplayXYZCloudCell::DisplayXYZCloudCell(Application& application_ref):
	Cell(),
	application_ref_(application_ref)
{
	parameters()["name"] = "DisplayXYZCloudCell";
}

planCloudsPtr_t DisplayXYZCloudCell::compute(planCloudsPtr_t planCloudListPtr)
{
	cell_name_ = boost::get<std::string>(parameters()["name"]);

	application_ref_.visualizer().add_xyz_clouds(planCloudListPtr);
	return planCloudListPtr;
}

