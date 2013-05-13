#include "application.h"
#include "displaycadmodelcell.h"
#include "plancloud.h"

DisplayCADModelCell::DisplayCADModelCell(Application& application_ref):
	Cell(),
	application_ref_(application_ref)
{
	parameters()["name"] = "DisplayCADModelCell";
}

planCloudsPtr_t DisplayCADModelCell::compute(planCloudsPtr_t planCloudListPtr)
{
	cell_name_ = boost::get<std::string>(parameters()["name"]);

	application_ref_.visualizer().add_cad_model(planCloudListPtr);
	return planCloudListPtr;
}
