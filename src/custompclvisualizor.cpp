# include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
# include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
# include <boost/thread/thread.hpp>

# include "custompclvisualizor.h"
# include "plancloud.h"
# include "typedefs.h"

CustomPCLVisualizor::CustomPCLVisualizor()
{

}

void CustomPCLVisualizor::visualize_hull_convex(pointCloudPtr_t pointCloudPtr)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	viewer->addPointCloud<pcl::PointXYZ> (pointCloudPtr, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->resetCameraViewpoint("sample cloud");

	//------------------------------------
	//-----Add shapes at cloud points-----
	//------------------------------------
	pointCloudPoints_t::size_type j = pointCloudPtr->points.size() - 1;
	std::string id = "";
	for(pointCloudPoints_t::size_type i = 0; i < pointCloudPtr->points.size(); ++i)
	{
		id = "Line" + boost::lexical_cast<std::string>(i);
		viewer->addLine<pcl::PointXYZ> (pointCloudPtr->points[j], pointCloudPtr->points[i], id);
		j = i;
	}

	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}

void CustomPCLVisualizor::display_cloud(pointCloudPtr_t pointCloudPtr)
{
	pcl::visualization::CloudViewer viewer("SimpleCloudViewer");
	viewer.showCloud(pointCloudPtr);
	while(!viewer.wasStopped())
	{
	}
}

void CustomPCLVisualizor::display_cloud(PlanCloud planCloud)
{
	pcl::visualization::CloudViewer viewer("SimpleCloudViewer");
	viewer.showCloud(planCloud.cloud());
	while(!viewer.wasStopped())
	{
	}
}

void CustomPCLVisualizor::display_all_clouds (planCloudsPtr_t planCloudListPtr)
{
	for(pointCloudPoints_t::size_type i = 0; i < planCloudListPtr->size(); i++)
	{
		display_cloud(planCloudListPtr->at(i).cloud());
	}
}

void CustomPCLVisualizor::display_all_clouds_together (planCloudsPtr_t planCloudListPtr)
{
	pointCloudPtr_t allPlansCloud = boost::make_shared<pointCloud_t >();

	for (pointCloudPoints_t::size_type i = 0;
		 i < planCloudListPtr->size(); i++)
	{
		*allPlansCloud += *(planCloudListPtr->at(i).cloud());
	}
	display_cloud(allPlansCloud);
}

void CustomPCLVisualizor::display_all_hull_convexes (planCloudsPtr_t planCloudListPtr)
{
	for (pointCloudPoints_t::size_type i = 0;
		 i < planCloudListPtr->size(); i++)
	{
		visualize_hull_convex(planCloudListPtr->at(i).cloud());
	}

}
