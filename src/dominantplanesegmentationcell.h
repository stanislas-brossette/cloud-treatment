#ifndef DOMINANTPLANESEGMENTATIONCELL_H
#define DOMINANTPLANESEGMENTATIONCELL_H

# include <pcl/segmentation/sac_segmentation.h>
# include <pcl/filters/extract_indices.h>

# include "cell.h"
# include "typedefs.h"
/// Implements a way to find the main plane in a scene and to separate it
/// from the rest of the cloud

class DominantPlaneSegmentationCell : public Cell
{
public:
	DominantPlaneSegmentationCell();
	virtual ~DominantPlaneSegmentationCell ()
	{
	}

	/// Finds the main plane and extracts its attached cloud while
	/// setting its modelCoefficient attribute and create a new cloud
	/// containing the remaining points of the cloud
	planCloudsPtr_t compute(planCloudsPtr_t);

private:
	double distance_threshold_;
	int max_iterations_;

	pcl::PointIndices::Ptr inliers_;
	pointCloudPtr_t initial_cloud_ptr_;
	planCloudPtr_t plan_cloud_ptr_;
	planCloudsPtr_t new_plan_cloud_list_ptr_;

	pcl::SACSegmentation<pcl::PointXYZ> seg_;
	pcl::ExtractIndices<pcl::PointXYZ> extract_;
};

#endif // DOMINANTPLANESEGMENTATIONCELL_H
