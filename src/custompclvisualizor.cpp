# include <string>
#include <boost/shared_ptr.hpp>
# include <pcl/visualization/pcl_visualizer.h>
# include <boost/thread/thread.hpp>

# include "custompclvisualizor.h"
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
