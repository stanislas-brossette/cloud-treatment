#include <boost/make_shared.hpp>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

#include "plancloud.h"
#include "planextractioncell.h"

PlanExtractionCell::PlanExtractionCell():
	Cell()
{
	parameters()["name"] = "PlanExtractionCell";
	parameters()["plan_rate"] = 0.1;
	parameters()["max_iteration"] = 100;
	parameters()["distance_threshold"] = 0.03;

	inliers_ = boost::make_shared<pcl::PointIndices>();
	seg_.setOptimizeCoefficients (true);
	seg_.setModelType (pcl::SACMODEL_PLANE);
	seg_.setMethodType (pcl::SAC_RANSAC);

	initial_cloud_ptr_ = boost::make_shared<pointCloud_t>();
	plan_cloud_ptr_ = boost::make_shared<PlanCloud > ();
	new_plan_cloud_list_ptr_ = boost::make_shared<planClouds_t >();
}

planCloudsPtr_t PlanExtractionCell::compute (planCloudsPtr_t planCloudListPtr)
{
	cell_name_ = boost::get<std::string>(parameters()["name"]);

	plan_rate_ = boost::get<double>(parameters()["plan_rate"]);
	distance_threshold_ = boost::get<double>(parameters()["distance_threshold"]);
	max_iteration_ = static_cast<int>(boost::get<double>(
						parameters()["max_iteration"]));

	seg_.setMaxIterations (max_iteration_);
	seg_.setDistanceThreshold (distance_threshold_);

	for(planClouds_t::size_type k=0; k<planCloudListPtr->size(); k++)
	{
		initial_cloud_ptr_ = planCloudListPtr->at(k).cloud();

		planClouds_t::size_type i = 0;
		planClouds_t::size_type nr_points = initial_cloud_ptr_->points.size ();

		while (initial_cloud_ptr_->points.size () > plan_rate_ * static_cast<double> (nr_points))
		{
			// Segment the largest planar component from the remaining cloud
			seg_.setInputCloud (initial_cloud_ptr_);
			seg_.segment (*inliers_, *(plan_cloud_ptr_->coefficients()));

			if (inliers_->indices.size () == 0)
			{
				throw std::runtime_error
						("Could not estimate a planar model for the given dataset.");
			}


			// Extract the inliers
			extract_.setInputCloud (initial_cloud_ptr_);
			extract_.setIndices (inliers_);
			extract_.setNegative (false);
			extract_.filter (*(plan_cloud_ptr_->cloud()));

			// Create the filtering object
			extract_.setNegative (true);
			extract_.filter (*initial_cloud_ptr_);

			new_plan_cloud_list_ptr_->push_back(*plan_cloud_ptr_);
			plan_cloud_ptr_ = boost::make_shared<PlanCloud > ();
			i++;
		}
	}

	return new_plan_cloud_list_ptr_;
}
