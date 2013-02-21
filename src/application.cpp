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
# include "filewritingcell.h"
# include "plancloud.h"
# include "typedefs.h"
# include "custompclvisualizor.h"


Application::Application()
{
	customPCLVisualizor = CustomPCLVisualizor();
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
	FileWritingCell fileWritingCell = FileWritingCell();

	std::string originPath = "../datafiles/";
	std::string fileName = "cloud17(Floor).pcd";
	fileCell.sync(originPath + fileName, planCloudListPtr);
//	customPCLVisualizor.display_all_clouds_together(planCloudListPtr);

	std::cout<<planCloudListPtr;
	planCloudListPtr = xyzSwitchCell.compute(planCloudListPtr);
//	customPCLVisualizor.display_all_clouds_together(planCloudListPtr);

	planCloudListPtr = filterCell.compute(planCloudListPtr);
//	customPCLVisualizor.display_all_clouds_together(planCloudListPtr);
	std::cout<<planCloudListPtr;
	planCloudListPtr = planExtractionCell.compute(planCloudListPtr);
//	customPCLVisualizor.display_all_clouds_together(planCloudListPtr);
	std::cout<<planCloudListPtr;
	planCloudListPtr = planProjectionCell.compute(planCloudListPtr);
//	customPCLVisualizor.display_all_clouds_together(planCloudListPtr);

	planCloudListPtr = hullConvexCell.compute(planCloudListPtr);
//	customPCLVisualizor.display_all_clouds_together(planCloudListPtr);
//	customPCLVisualizor.display_all_hull_convexes(planCloudListPtr);
	std::cout<<planCloudListPtr;
	fileWritingCell.write_files("../surffiles/", planCloudListPtr);
	std::cout<<planCloudListPtr;
}

void Application::display_all_coefficients (planCloudsPtr_t planCloudListPtr)
{
	for (pointCloudPoints_t::size_type i = 0;
		 i < planCloudListPtr->size(); i++)
	{
		std::cout<<planCloudListPtr->at(i)<<std::endl;
	}
}
