#include "application.h"
#include "displaykeypointcloudcell.h"
#include "plancloud.h"

DisplayKeypointCloudCell::DisplayKeypointCloudCell(Application& application_ref):
	Cell(),
	application_ref_(application_ref)
{
	parameters()["name"] = "DisplayKeypointCloudCell";
}

planCloudsPtr_t DisplayKeypointCloudCell::compute(planCloudsPtr_t planCloudListPtr)
{
	cell_name_ = boost::get<std::string>(parameters()["name"]);

	application_ref_.visualizer().add_keypoint_clouds(planCloudListPtr);
	return planCloudListPtr;
}


