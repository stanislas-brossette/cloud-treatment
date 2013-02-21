#ifndef CUSTOMPCLVISUALIZOR_H
#define CUSTOMPCLVISUALIZOR_H

#include "typedefs.h"
#include "plancloud.h"

class CustomPCLVisualizor
{
public:
	CustomPCLVisualizor();
	void visualize_hull_convex(pointCloudPtr_t);
	void display_cloud(pointCloudPtr_t);
	void display_cloud(PlanCloud planCloud);
	void display_all_clouds (planCloudsPtr_t planCloudListPtr);
	void display_all_clouds_together (planCloudsPtr_t planCloudListPtr);
	void display_all_hull_convexes (planCloudsPtr_t planCloudListPtr);
};

#endif // CUSTOMPCLVISUALIZOR_H
