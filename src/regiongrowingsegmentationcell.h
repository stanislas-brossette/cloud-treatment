#ifndef REGIONGROWINGSEGMENTATIONCELL_H
#define REGIONGROWINGSEGMENTATIONCELL_H

#include "typedefs.h"
#include "regiongrowingsegmentationcell.h"
#include "cell.h"
# include <boost/shared_ptr.hpp>
# include <pcl/PointIndices.h>
# include <pcl/filters/extract_indices.h>

/// \brief Implements the reading of a .pcl file and the creation of a
/// PointCloud object from it
class RegionGrowingSegmentationCell : public Cell
{
public:
	RegionGrowingSegmentationCell();
	virtual ~RegionGrowingSegmentationCell ()
	{
	}

	planCloudsPtr_t compute(planCloudsPtr_t);

private:
	pcl::ExtractIndices<pcl::PointXYZ> extract_;
	planCloudsPtr_t new_plan_cloud_list_ptr_;
	pointCloudPtr_t point_cloud_ptr_;
	planCloudPtr_t plan_cloud_ptr_;
	pcl::PointIndices::Ptr inliers_;
};

#endif // REGIONGROWINGSEGMENTATIONCELL_H
