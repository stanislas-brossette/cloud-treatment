#ifndef REGIONGROWINGSEGMENTATIONCELL_H
#define REGIONGROWINGSEGMENTATIONCELL_H

# include <boost/shared_ptr.hpp>
# include <pcl/PointIndices.h>
# include <pcl/filters/extract_indices.h>

#include "typedefs.h"
#include "regiongrowingsegmentationcell.h"
#include "cell.h"

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

	int number_of_neighbours_normal_estimation_;
	int min_cluster_size_;
	int max_cluster_size_;
	unsigned int number_of_neighbours_region_growing_;
	float smoothness_threshold_;
	float curvature_threshold_;
};

#endif // REGIONGROWINGSEGMENTATIONCELL_H
