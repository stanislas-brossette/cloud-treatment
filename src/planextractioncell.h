#ifndef PLANEXTRACTIONCELL_H
# define PLANEXTRACTIONCELL_H

# include <string>
# include <vector>

# include <boost/shared_ptr.hpp>
# include <pcl/filters/extract_indices.h>
# include <pcl/PointIndices.h>
# include <pcl/sample_consensus/method_types.h>
# include <pcl/sample_consensus/model_types.h>
# include <pcl/segmentation/sac_segmentation.h>

# include "cell.h"
# include "typedefs.h"
/// Implements a way to find the main plans in a scene and to extract
/// their "attached" clouds
class PlanExtractionCell : public Cell
{
public:
	PlanExtractionCell();
	virtual ~PlanExtractionCell ()
	{
	}

	/// Finds the main plans and extracts their attached clouds while
	/// setting their modelCoefficient attribute
	planCloudsPtr_t compute(planCloudsPtr_t);

	/// \name fine tuning of the algorithm
	/// \{
	void set_plan_rate (double plan_rate )
	{
		plan_rate_ = plan_rate;
	}
	void set_max_iterations (int max)
	{
		seg_.setMaxIterations (max);
	}
	void set_distance_threshold (double value)
	{
		seg_.setDistanceThreshold (value);
	}
	/// \}

private:
	/// plan_rate_ is the percentage of point that from which we'll stop
	/// looking for plans
	double plan_rate_;

	pcl::PointIndices::Ptr inliers_;
	pcl::SACSegmentation<pcl::PointXYZ> seg_;
	pcl::ExtractIndices<pcl::PointXYZ> extract_;
	pointCloudPtr_t initial_cloud_ptr_;
	planCloudPtr_t plan_cloud_ptr_;
	planCloudsPtr_t new_plan_cloud_list_ptr_;
	int max_iteration_;
	double distance_threshold_;
};

#endif // PLANEXTRACTIONCELL_H
