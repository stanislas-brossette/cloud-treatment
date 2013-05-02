#include <boost/make_shared.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "dominantplanesegmentationcell.h"
#include "plancloud.h"

DominantPlaneSegmentationCell::DominantPlaneSegmentationCell():
	Cell()
{
	parameters()["name"] = "DominantPlaneSegmentationCell";
	parameters()["max_iteration"] = 100;
	parameters()["distance_threshold"] = 0.03;

	seg_.setOptimizeCoefficients (false);
	seg_.setModelType (pcl::SACMODEL_PLANE);
	seg_.setMethodType (pcl::SAC_RANSAC);

	initial_cloud_ptr_ = boost::make_shared<pointCloud_t>();
	plan_cloud_ptr_ = boost::make_shared<PlanCloud > ();
	new_plan_cloud_list_ptr_ = boost::make_shared<planClouds_t >();
}

planCloudsPtr_t DominantPlaneSegmentationCell::compute
(planCloudsPtr_t planCloudListPtr)
{
	cell_name_ = boost::get<std::string>(parameters()["name"]);

	distance_threshold_ = boost::get<double>(
				parameters()["distance_threshold"]);
	max_iterations_ = static_cast<int>(
				boost::get<double>(parameters()["max_iteration"]));

	seg_.setDistanceThreshold (distance_threshold_);
	seg_.setMaxIterations (max_iterations_);

	for(std::size_t j = 0;j<planCloudListPtr->size(); ++j)
	{
		// Segmentation of the dominant plane
		initial_cloud_ptr_ = planCloudListPtr->at(j).cloud();

		seg_.setInputCloud (initial_cloud_ptr_);

		pcl::ModelCoefficients::Ptr coefficients (
					new pcl::ModelCoefficients ());
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
		seg_.segment (*inliers, *coefficients);

		// Extract the inliers
		extract_.setInputCloud (initial_cloud_ptr_);
		extract_.setIndices (inliers);

		extract_.setNegative (true);
		extract_.filter (*(plan_cloud_ptr_->cloud()));
		new_plan_cloud_list_ptr_->push_back(*plan_cloud_ptr_);

//		extract_.setNegative (false);
//		extract_.filter (*(plan_cloud_ptr_->cloud()));
//		new_plan_cloud_list_ptr_->push_back(*plan_cloud_ptr_);
	}
	return new_plan_cloud_list_ptr_;
}

