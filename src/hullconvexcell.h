#ifndef HULLCONVEXCELL_H
#define HULLCONVEXCELL_H

#include <pcl/surface/convex_hull.h>

# include "typedefs.h"
# include "cell.h"

///Implements a method to compute the Hull Convex of the clouds
class HullConvexCell : public Cell
{
public:
	HullConvexCell();
	planCloudsPtr_t compute(planCloudsPtr_t);
private:
	pcl::ConvexHull<pcl::PointXYZ> chull;
	pointCloudPtr_t point_cloud_ptr_;
};

#endif // HULLCONVEXCELL_H
