#include "application.h"
#include "displayconvexcloudcell.h"
#include "plancloud.h"

DisplayConvexCloudCell::DisplayConvexCloudCell(Application& application_ref):
	Cell(),
	application_ref_(application_ref)
{
	parameters()["name"] = "DisplayConvexCloudCell";
}

planCloudsPtr_t DisplayConvexCloudCell::compute(planCloudsPtr_t planCloudListPtr)
{
	cell_name_ = boost::get<std::string>(parameters()["name"]);

	application_ref_.visualizer().add_convex_clouds(planCloudListPtr);
	return planCloudListPtr;
}


