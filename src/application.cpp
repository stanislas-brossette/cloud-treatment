#include <vector>
#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <pcl/visualization/cloud_viewer.h>

# include "application.h"
# include "filecell.h"
# include "xyzswitchcell.h"
# include "filtercell.h"
# include "planextractioncell.h"
# include "planprojectioncell.h"
# include "hullconvexcell.h"
# include "plancloud.h"
# include "typedefs.h"
# include "custompclvisualizor.h"


Application::Application()
{
}

void Application::Run()
{
	planCloudsPtr_t planCloudListPtr = boost::make_shared < planClouds_t > ();

	FileCell fileCell = FileCell();
	FilterCell filterCell = FilterCell();
	XYZSwitchCell xyzSwitchCell = XYZSwitchCell();
	PlanExtractionCell planExtractionCell = PlanExtractionCell();
	PlanProjectionCell planProjectionCell = PlanProjectionCell();
	HullConvexCell hullConvexCell = HullConvexCell();

	std::string originPath = "../datafiles/";
	std::string fileName = "cloud17(Floor).pcd";
	fileCell.sync(originPath + fileName, planCloudListPtr);

	planCloudListPtr = xyzSwitchCell.compute(planCloudListPtr);

	planCloudListPtr = filterCell.compute(planCloudListPtr);
	display_all_clouds_together(planCloudListPtr);

	planCloudListPtr = planExtractionCell.compute(planCloudListPtr);

	planCloudListPtr = planProjectionCell.compute(planCloudListPtr);
	display_all_clouds_together(planCloudListPtr);

	planCloudListPtr = hullConvexCell.compute(planCloudListPtr);

	display_all_hull_convexes(planCloudListPtr);
	display_all_coefficients(planCloudListPtr);
}

void Application::display_all_clouds (planCloudsPtr_t planCloudListPtr)
{
	for(pointCloudPoints_t::size_type i = 0; i < planCloudListPtr->size(); i++)
	{
		planCloudListPtr->at(i).display_cloud();
	}
}

void Application::display_all_clouds_together (planCloudsPtr_t planCloudListPtr)
{
	pointCloudPtr_t allPlansCloud = boost::make_shared<pointCloud_t >();

	for (pointCloudPoints_t::size_type i = 0;
		 i < planCloudListPtr->size(); i++)
	{
		*allPlansCloud += *(planCloudListPtr->at(i).cloud());
	}
	pcl::visualization::CloudViewer viewer("SimpleCloudViewer");
	viewer.showCloud(allPlansCloud);
	while(!viewer.wasStopped())
	{
	}
}

void Application::display_all_coefficients (planCloudsPtr_t planCloudListPtr)
{
	for (pointCloudPoints_t::size_type i = 0;
		 i < planCloudListPtr->size(); i++)
	{
		planCloudListPtr->at(i).display_planar_components();
	}
}


void Application::display_all_hull_convexes (planCloudsPtr_t planCloudListPtr)
{
	CustomPCLVisualizor customPCLVisualizor = CustomPCLVisualizor();
	for (pointCloudPoints_t::size_type i = 0;
		 i < planCloudListPtr->size(); i++)
	{
		customPCLVisualizor.visualize_hull_convex(planCloudListPtr->at(i).cloud());
	}

}
