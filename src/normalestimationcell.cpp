#include <boost/make_shared.hpp>

#include <pcl/features/integral_image_normal.h>

#include "normalestimationcell.h"


NormalEstimationCell::NormalEstimationCell():
	Cell(),
	point_cloud_ptr_(boost::make_shared<pointCloud_t > ())
{
	parameters()["name"] = "NormalEstimationCell";
	parameters()["number_of_neighbours_normal_estimation"] = 10;
}

planCloudsPtr_t NormalEstimationCell::compute(planCloudsPtr_t planCloudListPtr)
{
	cell_name_ = boost::get<std::string>(parameters()["name"]);

	number_of_neighbours_normal_estimation_ =
			static_cast<int>(boost::get<double>(
				parameters()["number_of_neighbours_normal_estimation"]));

	for(pointCloudPoints_t::size_type j = 0; j<planCloudListPtr->size(); ++j)
	{
		point_cloud_ptr_ = planCloudListPtr->at(j).cloud();
		pcl::search::Search<pcl::PointXYZ>::Ptr tree =
				boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >
				(new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
		normal_estimator.setSearchMethod (tree);
		normal_estimator.setInputCloud (point_cloud_ptr_);
		normal_estimator.setKSearch (number_of_neighbours_normal_estimation_);
		normal_estimator.compute (*(planCloudListPtr->at(j).normals()));

		/// The following commented method is supposed to be faster but requires
		/// the cloud to be organized.
		/// Will be implemented in the futur
//		pcl::IntegralImageNormalEstimation<pointT, pcl::Normal> norm_est;
//		// Specify method for normal estimation
//		norm_est.setNormalEstimationMethod (norm_est.AVERAGE_3D_GRADIENT);
//		// Specify max depth change factor
//		norm_est.setMaxDepthChangeFactor(0.02f);
//		// Specify smoothing area size
//		norm_est.setNormalSmoothingSize(10.0f);
//		// Set the input points
//		norm_est.setInputCloud (point_cloud_ptr_);
//		// Estimate the surface normals and
//		// store the result in "normals_out"
//		norm_est.compute (*(planCloudListPtr->at(j).normals()));

	}

	return planCloudListPtr;
}
