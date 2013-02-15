#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "application.h"
#include "filecell.h"
#include "filtercell.h"


Application::Application()
{
}

void Application::Run()
{
	boost::shared_ptr<std::vector<PlanCloud> > planCloudListPtr = boost::make_shared<std::vector<PlanCloud> >();

	FileCell fileCell = FileCell();
	FilterCell filterCell = FilterCell();

	std::string originPath = "/home/stanislas/coding/PCLTests/PointCloudFiles/";
	std::string fileName = "cloud17(Floor).pcd";
	fileCell.sync(originPath + fileName, planCloudListPtr);
	std::cout<<"Original PointCloud has: "
			<< (planCloudListPtr->at(0).cloud())->points.size () << " data points." << std::endl;

	planCloudListPtr = filterCell.compute(planCloudListPtr);

}

