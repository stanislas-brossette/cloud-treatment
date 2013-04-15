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
	point_cloud_ptr_ = boost::make_shared<pointCloud_t > ();
	inliers_ = boost::make_shared<pcl::PointIndices>();
	new_plan_cloud_list_ptr_ = boost::make_shared<planClouds_t >();
	plan_cloud_ptr_ = boost::make_shared<PlanCloud > ();
}

planCloudsPtr_t RegionGrowingSegmentationCell::compute(planCloudsPtr_t planCloudListPtr)
{
	for(pointCloudPoints_t::size_type j = 0; j<planCloudListPtr->size(); ++j)
	{
		point_cloud_ptr_ = planCloudListPtr->at(j).cloud();
		pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
		normal_estimator.setSearchMethod (tree);
		normal_estimator.setInputCloud (point_cloud_ptr_);
		normal_estimator.setKSearch (50);
		normal_estimator.compute (*normals);

//		pcl::IndicesPtr indices (new std::vector <int>);
//		pcl::PassThrough<pcl::PointXYZ> pass;
//		pass.setInputCloud (cloud);
//		pass.setFilterFieldName ("z");
//		pass.setFilterLimits (0.0, 1.0);
//		pass.filter (*indices);

		pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
		reg.setMinClusterSize (500);
		reg.setMaxClusterSize (70000);
		reg.setSearchMethod (tree);
		reg.setNumberOfNeighbours (30);
		reg.setInputCloud (point_cloud_ptr_);
		//reg.setIndices (indices);
		reg.setInputNormals (normals);
		reg.setSmoothnessThreshold (static_cast<float>(6.0 / 180.0 * M_PI));
		reg.setCurvatureThreshold (1.0);

		std::vector <pcl::PointIndices> clusters;
		reg.extract (clusters);

		std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
		std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
		std::cout << "These are the indices of the points of the initial" <<
					 std::endl << "cloud that belong to the first cluster:" << std::endl;
		unsigned int counter = 0;
		while (counter < 5 || counter > clusters[0].indices.size())
		{
			std::cout << clusters[0].indices[counter] << std::endl;
			counter++;
		}

		pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
		pcl::visualization::CloudViewer viewer ("Cluster viewer");
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
			std::cout << "treating cluster "<< j << " made of " << plan_cloud_ptr_->cloud()->size() << "points" << std::endl;
			new_plan_cloud_list_ptr_->push_back(*plan_cloud_ptr_);
			plan_cloud_ptr_ = boost::make_shared<PlanCloud > ();
		}

	}
	return new_plan_cloud_list_ptr_;
}
