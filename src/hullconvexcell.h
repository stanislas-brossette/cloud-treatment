#ifndef HULLCONVEXCELL_H
#define HULLCONVEXCELL_H

# include <pcl/surface/convex_hull.h>

# include "typedefs.h"
# include "cell.h"

/// \brief Implements a method to compute the Hull Convexes of the clouds
class HullConvexCell : public Cell
{
public:
	HullConvexCell();
	virtual ~HullConvexCell ()
	{
	}
	/// Computes the hull convex of each of the clouds in the list
	/// \pre the cloud list must have been through the PlanExtractionCell
	/// algorithm
	planCloudsPtr_t compute(planCloudsPtr_t);
private:
	/// PCL made HullConvex generator object
	pcl::ConvexHull<pcl::PointXYZ> chull;
	pointCloudPtr_t point_cloud_ptr_;
};

#endif // HULLCONVEXCELL_H
