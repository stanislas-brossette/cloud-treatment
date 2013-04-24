#include "application.h"
#include "displayxyzcloudcell.h"
#include "plancloud.h"

DisplayXYZCloudCell::DisplayXYZCloudCell(Application& application_ref)
	:application_ref_(application_ref)
{
	cell_name() = "DisplayXYZCloudCell";
}

planCloudsPtr_t DisplayXYZCloudCell::compute(planCloudsPtr_t planCloudListPtr)
{
	application_ref_.visualizer().add_xyz_clouds(planCloudListPtr);
	return planCloudListPtr;
}

