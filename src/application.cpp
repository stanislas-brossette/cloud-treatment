#include <vector>
#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <pcl/visualization/cloud_viewer.h>

#include "application.h"
#include "filecell.h"
#include "xyzswitchcell.h"
#include "filtercell.h"
#include "planextractioncell.h"
#include "plancloud.h"


Application::Application()
{
}

void Application::Run()
{
	boost::shared_ptr<std::vector<PlanCloud> > planCloudListPtr = boost::make_shared<std::vector<PlanCloud> >();

	FileCell fileCell = FileCell();
	FilterCell filterCell = FilterCell();
	XYZSwitchCell xyzSwitchCell = XYZSwitchCell();
	PlanExtractionCell planExtractionCell = PlanExtractionCell();


	std::string originPath = "../datafiles/";
	std::string fileName = "cloud17(Floor).pcd";
	fileCell.sync(originPath + fileName, planCloudListPtr);
//	planCloudListPtr->at(0).info();
//	planCloudListPtr->at(0).display();

	planCloudListPtr = xyzSwitchCell.compute(planCloudListPtr);
//	planCloudListPtr->at(0).info();
//	planCloudListPtr->at(0).display();

	planCloudListPtr = filterCell.compute(planCloudListPtr);
//	planCloudListPtr->at(0).info();
//	planCloudListPtr->at(0).display();

	planCloudListPtr = planExtractionCell.compute(planCloudListPtr);

	display_all_clouds_together(planCloudListPtr);
	display_all_coefficients(planCloudListPtr);
}

void Application::display_all_clouds(boost::shared_ptr<std::vector<PlanCloud> > planCloudListPtr)
{
	for(int i = 0; i < planCloudListPtr->size(); i++)
	{
		planCloudListPtr->at(i).display_cloud();
	}
}

void Application::display_all_clouds_together(boost::shared_ptr<std::vector<PlanCloud> > planCloudListPtr)
{
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > allPlansCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for (int i = 0; i < planCloudListPtr->size(); i++)
	{
		*allPlansCloud += *(planCloudListPtr->at(i).cloud());
	}
	pcl::visualization::CloudViewer viewer("SimpleCloudViewer");
	viewer.showCloud(allPlansCloud);
	while(!viewer.wasStopped())
	{
	}
}

void Application::display_all_coefficients(boost::shared_ptr<std::vector<PlanCloud> > planCloudListPtr)
{
	for (int i = 0; i < planCloudListPtr->size(); i++)
	{
		planCloudListPtr->at(i).display_planar_components();
	}
}
