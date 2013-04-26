#include "application.h"
#include "displaynormalcloudcell.h"
#include "plancloud.h"

DisplayNormalCloudCell::DisplayNormalCloudCell(Application& application_ref):
	Cell("DisplayNormalCloudCell"),
	application_ref_(application_ref)
{
}

planCloudsPtr_t DisplayNormalCloudCell::compute(planCloudsPtr_t planCloudListPtr)
{
	application_ref_.visualizer().add_normals_clouds(planCloudListPtr);
	return planCloudListPtr;
}
