#ifndef CUSTOMPCLVISUALIZOR_H
#define CUSTOMPCLVISUALIZOR_H

#include "plancloud.h"
#include "typedefs.h"

/// \brief Visualization class
///
/// This class offers different method to visualize the plan clouds in different ways

class CustomPCLVisualizor
{
public:
	CustomPCLVisualizor();
	virtual ~CustomPCLVisualizor ()
	{
	}

	/// Visualize a hull convex
	///
	/// \pre the pointCloud that is given to this method must be a hull
	/// convex, otherwise, the visualization won't make sense.
	void visualize_hull_convex(pointCloudPtr_t);

	/// visualize all the hull convex in the list, one by one
	void display_all_hull_convexes (planCloudsPtr_t planCloudListPtr);

	/// \name Simple displays
	/// displays a pointCloud in the simplest way
	/// \{
	void display_cloud(pointCloudPtr_t);
	void display_cloud(PlanCloud planCloud);
	/// \}

	/// display all the clouds in the list one by one
	void display_all_clouds (planCloudsPtr_t planCloudListPtr);

	/// display all the clouds in the list together
	void display_all_clouds_together (planCloudsPtr_t planCloudListPtr);

	void display_cloud_color(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> >);
};

#endif // CUSTOMPCLVISUALIZOR_H
