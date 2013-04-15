#include <vector>
#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "application.h"
#include "filecell.h"
#include "xyzswitchcell.h"
#include "filtercell.h"
#include "regiongrowingsegmentationcell.h"
#include "planextractioncell.h"
#include "planprojectioncell.h"
#include "hullconvexcell.h"
#include "filewritingcell.h"
#include "orientationcell.h"
#include "plancloud.h"
#include "typedefs.h"
#include "custompclvisualizor.h"
#include "visualizer.h"


Application::Application()
{
	customPCLVisualizor = CustomPCLVisualizor();
	pcd_file_name = "cloud17(Floor)";
}

Application::Application(std::string path)
{
	customPCLVisualizor = CustomPCLVisualizor();
	pcd_file_name = path;
}

void Application::Run()
{
	planCloudsPtr_t planCloudListPtr = boost::make_shared < planClouds_t > ();

	FileCell fileCell = FileCell();
	FilterCell filterCell = FilterCell();
	RegionGrowingSegmentationCell regionGrowingSegmentationCell = RegionGrowingSegmentationCell();
	XYZSwitchCell xyzSwitchCell = XYZSwitchCell();
	PlanExtractionCell planExtractionCell = PlanExtractionCell();
	PlanProjectionCell planProjectionCell = PlanProjectionCell();
	HullConvexCell hullConvexCell = HullConvexCell();
	OrientationCell orientationCell = OrientationCell();
	FileWritingCell fileWritingCell = FileWritingCell();
	Visualizer visualizer = Visualizer();



//	customPCLVisualizor.display_cloud_color(fileCell.sync_color(pcd_file_name));
	visualizer.add_xyzrgba_clouds(fileCell.sync_color(pcd_file_name));
	fileCell.sync(pcd_file_name, planCloudListPtr);
	std::cout<<"FileCell completed"<<std::endl<<planCloudListPtr;
//	customPCLVisualizor.display_all_clouds_together(planCloudListPtr);


	planCloudListPtr = filterCell.compute(planCloudListPtr);
	std::cout<<"filterCell completed"<<std::endl<<planCloudListPtr;
//	visualizer.add_xyz_clouds(planCloudListPtr);
//	customPCLVisualizor.display_all_clouds_together(planCloudListPtr);

//	fileWritingCell.write_cloud_files("../treatedCloudFiles/", pcd_file_name, planCloudListPtr );
//	customPCLVisualizor.display_all_clouds_together(planCloudListPtr);

	planCloudListPtr = filterCell.computePassThrough(planCloudListPtr, "z", 0.0f, 2.7f);
//	planCloudListPtr = filterCell.computePassThrough(planCloudListPtr, "y", -1.0f, 0.6f);
//	planCloudListPtr = filterCell.computePassThrough(planCloudListPtr, "x", -1.0f, 1.0f);
	std::cout<<"filterCellPassThrough completed"<<std::endl<<planCloudListPtr;
//	customPCLVisualizor.display_all_clouds_together(planCloudListPtr);
//	visualizer.add_xyz_clouds(planCloudListPtr);
//	fileWritingCell.write_cloud_files("../treatedCloudFiles/", pcd_file_name, planCloudListPtr );

	planCloudListPtr = regionGrowingSegmentationCell.compute(planCloudListPtr);
//	std::cout<<"regionGrowingSegmentationCell completed"<<std::endl<<planCloudListPtr;
//	visualizer.add_xyz_clouds(planCloudListPtr);

//	planCloudListPtr = xyzSwitchCell.compute(planCloudListPtr);
//	customPCLVisualizor.display_all_clouds_together(planCloudListPtr);

	planCloudListPtr = planExtractionCell.compute(planCloudListPtr);
//	std::cout<<"planExtractionCell completed"<<std::endl<<planCloudListPtr;
//	customPCLVisualizor.display_all_clouds_together(planCloudListPtr);
//	visualizer.add_xyz_clouds(planCloudListPtr);

	planCloudListPtr = planProjectionCell.compute(planCloudListPtr);
//	customPCLVisualizor.display_all_clouds(planCloudListPtr);
//	customPCLVisualizor.display_all_clouds_together(planCloudListPtr);
//	visualizer.add_xyz_clouds(planCloudListPtr);

	planCloudListPtr = hullConvexCell.compute(planCloudListPtr);
//	std::cout<<"hullConvexCell completed"<<std::endl<<planCloudListPtr;
//	customPCLVisualizor.display_all_clouds_together(planCloudListPtr);
//	customPCLVisualizor.display_all_hull_convexes(planCloudListPtr);

	visualizer.add_convex_clouds(planCloudListPtr);
	visualizer.display_all();

//	planCloudListPtr = orientationCell.compute(planCloudListPtr);
//	std::cout<<"orientationCell completed"<<std::endl<<planCloudListPtr;

//	planCloudListPtr = fileWritingCell.write_files("../surffiles/", planCloudListPtr, pcd_file_name);
//	std::cout<<Verbose(1)<<planCloudListPtr;

}

