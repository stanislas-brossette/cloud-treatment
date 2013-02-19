#ifndef PLANPROJECTIONCELL_H
# define PLANPROJECTIONCELL_H

# include <pcl/filters/project_inliers.h>

# include "typedefs.h"
# include "cell.h"

///Implements the projection of the point clouds on their carrying plans
class PlanProjectionCell : public Cell
{
public:
	PlanProjectionCell();
	planCloudsPtr_t compute(planCloudsPtr_t);
private:
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	planCloudPtr_t plan_cloud_ptr_;
};

#endif // PLANPROJECTIONCELL_H