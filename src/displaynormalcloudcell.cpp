#include "application.h"
#include "displaynormalcloudcell.h"
#include "plancloud.h"

DisplayNormalCloudCell::DisplayNormalCloudCell(Application& application_ref):
	Cell(),
	application_ref_(application_ref)
{
	parameters()["name"] = "DisplayNormalCloudCell";
}

planCloudsPtr_t DisplayNormalCloudCell::compute(planCloudsPtr_t planCloudListPtr)
{
	cell_name_ = boost::get<std::string>(parameters()["name"]);

	application_ref_.visualizer().add_normals_clouds(planCloudListPtr);
	return planCloudListPtr;
}
