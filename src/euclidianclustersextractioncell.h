#ifndef EUCLIDIANCLUSTERSEXTRACTIONCELL_H
#define EUCLIDIANCLUSTERSEXTRACTIONCELL_H

#include <pcl/segmentation/extract_clusters.h>

#include "cell.h"

/// \brief Implements a voxel grid filter that reduces the number of points
/// in the treated cloud
class EuclidianClustersExtractionCell : public Cell
{
public:
	/// \name Constructors
	/// \{
	EuclidianClustersExtractionCell();
	/// \}

	/// Use EuclidieanClusterExtraction to group a cloud into contiguous clusters
	/// Inputs:
	/// input
	/// The input point cloud
	/// cluster_tolerance
	/// The maximum distance between neighboring points in a cluster
	/// min/max_cluster_size
	/// The minimum and maximum allowable cluster sizes

	virtual ~EuclidianClustersExtractionCell ()
	{
	}

	planCloudsPtr_t compute(planCloudsPtr_t);

private:
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidian_cluster_extractor_;

	planCloudsPtr_t new_plan_cloud_list_ptr_;
	planCloudPtr_t plan_cloud_ptr_;

	double cluster_tolerance_;
	int min_cluster_size_;
	int max_cluster_size_;
};

#endif // EUCLIDIANCLUSTERSEXTRACTIONCELL_H
