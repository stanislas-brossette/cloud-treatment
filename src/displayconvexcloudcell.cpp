#include "application.h"
#include "displayconvexcloudcell.h"
#include "plancloud.h"

DisplayConvexCloudCell::DisplayConvexCloudCell(Application& application_ref)
	:application_ref_(application_ref)
{
	cell_name() = "DisplayXYZCloudCell";
}

planCloudsPtr_t DisplayConvexCloudCell::compute(planCloudsPtr_t planCloudListPtr)
{
	application_ref_.visualizer().add_convex_clouds(planCloudListPtr);
	return planCloudListPtr;
}


