#include <iostream>
#include <vector>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>

#include "regiongrowingsegmentationcell.h"

RegionGrowingSegmentationCell::RegionGrowingSegmentationCell():
	Cell()
{
	parameters()["name"] = "RegionGrowingSegmentationCell";
	parameters()["number_of_neighbours_normal_estimation"] = 50;
	parameters()["min_cluster_size"] = 500;
	parameters()["max_cluster_size"] = 70000;
	parameters()["number_of_neighbours_region_growing"] = 30;
	parameters()["smoothness_threshold"] = 6;
	parameters()["curvature_threshold"] = 1;

	point_cloud_ptr_ = boost::make_shared<pointCloud_t > ();
	inliers_ = boost::make_shared<pcl::PointIndices>();
	new_plan_cloud_list_ptr_ = boost::make_shared<planClouds_t >();
	plan_cloud_ptr_ = boost::make_shared<PlanCloud > ();
}

planCloudsPtr_t RegionGrowingSegmentationCell::compute(planCloudsPtr_t planCloudListPtr)
{
	cell_name_ = boost::get<std::string>(parameters()["name"]);

	number_of_neighbours_normal_estimation_ = static_cast<int>(boost::get<double>(
		parameters()["number_of_neighbours_normal_estimation"]));
	min_cluster_size_ = static_cast<int>(boost::get<double>(
							parameters()["min_cluster_size"]));
	max_cluster_size_ = static_cast<int>(boost::get<double>(
							parameters()["max_cluster_size"]));
	number_of_neighbours_region_growing_ = static_cast<unsigned int>(
		boost::get<double>(parameters()["number_of_neighbours_region_growing"]));
	smoothness_threshold_ = static_cast<float>(boost::get<double>(
								parameters()["smoothness_threshold"]));
	curvature_threshold_ = static_cast<float>(boost::get<double>(
								parameters()["curvature_threshold"]));

	for(pointCloudPoints_t::size_type j = 0; j<planCloudListPtr->size(); ++j)
	{
		point_cloud_ptr_ = planCloudListPtr->at(j).cloud();
		pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
		normal_estimator.setSearchMethod (tree);
		normal_estimator.setInputCloud (point_cloud_ptr_);
		normal_estimator.setKSearch (number_of_neighbours_normal_estimation_);
		normal_estimator.compute (*normals);

		pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
		reg.setMinClusterSize (min_cluster_size_);
		reg.setMaxClusterSize (max_cluster_size_);
		reg.setSearchMethod (tree);
		reg.setNumberOfNeighbours (number_of_neighbours_region_growing_);
		reg.setInputCloud (point_cloud_ptr_);
		//reg.setIndices (indices);
		reg.setInputNormals (normals);
		reg.setSmoothnessThreshold (static_cast<float>(smoothness_threshold_ / 180.0 * M_PI));
		reg.setCurvatureThreshold (curvature_threshold_);

		std::vector <pcl::PointIndices> clusters;
		reg.extract (clusters);

//		pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
//		pcl::visualization::CloudViewer viewer ("Cluster viewer");
//		viewer.showCloud(colored_cloud);
//		while (!viewer.wasStopped ())
//		{
//		}

		for(unsigned int j = 0; j<clusters.size(); j++)
		{
			// Extract the inliers
			*inliers_ = clusters[j];
			extract_.setInputCloud (point_cloud_ptr_);
			extract_.setIndices (inliers_);
			extract_.setNegative (false);
			extract_.filter (*(plan_cloud_ptr_->cloud()));
//			std::cout << "treating cluster "<< j << " made of " << plan_cloud_ptr_->cloud()->size() << "points" << std::endl;
			new_plan_cloud_list_ptr_->push_back(*plan_cloud_ptr_);
			plan_cloud_ptr_ = boost::make_shared<PlanCloud > ();
		}
	}
	return new_plan_cloud_list_ptr_;
}
