#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "application.h"
#include "filecell.h"
#include "xyzswitchcell.h"
#include "filtercell.h"


Application::Application()
{
}

void Application::Run()
{
	boost::shared_ptr<std::vector<PlanCloud> > planCloudListPtr = boost::make_shared<std::vector<PlanCloud> >();

	FileCell fileCell = FileCell();
	FilterCell filterCell = FilterCell();
	XYZSwitchCell xyzSwitchCell = XYZSwitchCell();

	std::string originPath = "/home/stanislas/coding/PCLTests/PointCloudFiles/";
	std::string fileName = "cloud17(Floor).pcd";
	fileCell.sync(originPath + fileName, planCloudListPtr);
	planCloudListPtr->at(0).info();
	planCloudListPtr->at(0).display();

	planCloudListPtr = xyzSwitchCell.compute(planCloudListPtr);
	planCloudListPtr->at(0).info();
	planCloudListPtr->at(0).display();

	planCloudListPtr = filterCell.compute(planCloudListPtr);

}

