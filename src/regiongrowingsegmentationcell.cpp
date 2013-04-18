#include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/PointIndices.h>
# include <pcl/filters/extract_indices.h>

#include "regiongrowingsegmentationcell.h"

RegionGrowingSegmentationCell::RegionGrowingSegmentationCell()
{
	cell_name() = "RegionGrowingSegmentationCell";

	parameters()["number_of_neighbours_normal_estimation_"] = 50;
	parameters()["min_cluster_size_"] = 500;
	parameters()["max_cluster_size_"] = 70000;
	parameters()["number_of_neighbours_region_growing_"] = 30;
	parameters()["smoothness_threshold_"] = 6;
	parameters()["curvature_threshold_"] = 1;

	point_cloud_ptr_ = boost::make_shared<pointCloud_t > ();
	inliers_ = boost::make_shared<pcl::PointIndices>();
	new_plan_cloud_list_ptr_ = boost::make_shared<planClouds_t >();
	plan_cloud_ptr_ = boost::make_shared<PlanCloud > ();
}

planCloudsPtr_t RegionGrowingSegmentationCell::compute(planCloudsPtr_t planCloudListPtr)
{

	number_of_neighbours_normal_estimation_ = static_cast<int>(parameters()["number_of_neighbours_normal_estimation_"]);
	min_cluster_size_ = static_cast<int>(parameters()["min_cluster_size_"]);
	max_cluster_size_ = static_cast<int>(parameters()["max_cluster_size_"]);
	number_of_neighbours_region_growing_ = static_cast<int>(parameters()["number_of_neighbours_region_growing_"]);
	smoothness_threshold_ = static_cast<float>(parameters()["smoothness_threshold_"]);
	curvature_threshold_ = static_cast<float>(parameters()["curvature_threshold_"]);

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
