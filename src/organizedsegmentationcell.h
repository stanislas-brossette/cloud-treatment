#ifndef ORGANIZEDSEGMENTATIONCELL_H
#define ORGANIZEDSEGMENTATIONCELL_H

#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "cell.h"
#include "typedefs.h"

class OrganizedSegmentationCell : public Cell
{
public:
	OrganizedSegmentationCell();

	virtual ~OrganizedSegmentationCell ()
	{
	}

	planCloudsPtr_t compute(planCloudsPtr_t);

private:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_;
	pcl::IntegralImageNormalEstimation<pointT, pcl::Normal> ne;
	pcl::OrganizedMultiPlaneSegmentation<pointT, pcl::Normal, pcl::Label> mps;
	pcl::PlaneCoefficientComparator<pointT, pcl::Normal>::Ptr plane_comparator_;
	pcl::EuclideanPlaneCoefficientComparator<pointT, pcl::Normal>::Ptr euclidean_comparator_;
//	pcl::RGBPlaneCoefficientComparator<pointT, pcl::Normal>::Ptr rgb_comparator_;
	pcl::EdgeAwarePlaneComparator<pointT, pcl::Normal>::Ptr edge_aware_comparator_;
	pcl::EuclideanClusterComparator<pointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_;

	int display_normals_;
	int display_curvature_;
	int display_distance_map_;

	int use_planar_refinement_;
	int use_clustering_;

	uint plane_min_inliers_;
	float plane_angular_threshold_;
	double plane_distance_threshold_;

	float euclidian_cluster_distance_threshold_;
	uint euclidian_cluster_min_size_;

	float edge_aware_distance_threshold_;


};

#endif // ORGANIZEDSEGMENTATIONCELL_H
