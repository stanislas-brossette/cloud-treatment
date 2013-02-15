#ifndef PLANEXTRACTIONCELL_H
#define PLANEXTRACTIONCELL_H

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>

#include "cell.h"

class PlanExtractionCell : public Cell
{
public:
	PlanExtractionCell();
	boost::shared_ptr<std::vector<PlanCloud> > compute(boost::shared_ptr<std::vector<PlanCloud> >);

private:
	/// plan_rate_ is the percentage of point that from which we'll stop looking for plans
	double plan_rate_;

	pcl::PointIndices::Ptr inliers_;
	pcl::SACSegmentation<pcl::PointXYZ> seg_;
	pcl::ExtractIndices<pcl::PointXYZ> extract_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr initial_cloud_ptr_;
	boost::shared_ptr<PlanCloud > plan_cloud_ptr_;
	boost::shared_ptr<std::vector<PlanCloud> > new_plan_cloud_list_ptr_;
};

#endif // PLANEXTRACTIONCELL_H
