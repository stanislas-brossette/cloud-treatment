#include "application.h"
#include "displaykeypointcloudcell.h"
#include "plancloud.h"

DisplayKeypointCloudCell::DisplayKeypointCloudCell(Application& application_ref):
	Cell("DisplayKeypointCloudCell"),
	application_ref_(application_ref)
{
}

planCloudsPtr_t DisplayKeypointCloudCell::compute(planCloudsPtr_t planCloudListPtr)
{
	application_ref_.visualizer().add_keypoint_clouds(planCloudListPtr);
	return planCloudListPtr;
}


