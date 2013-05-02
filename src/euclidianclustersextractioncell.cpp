#include "boost/make_shared.hpp"

#include "euclidianclustersextractioncell.h"
#include "plancloud.h"

EuclidianClustersExtractionCell::EuclidianClustersExtractionCell():
	Cell()
{
	parameters()["name"] = "EuclidianClustersExtractionCell";
	parameters()["cluster_tolerance"] = 0.1;
	parameters()["min_cluster_size"] = 100;
	parameters()["max_cluster_size"] = 100000;

	new_plan_cloud_list_ptr_ = boost::make_shared<planClouds_t >();
}

planCloudsPtr_t EuclidianClustersExtractionCell::compute(
		planCloudsPtr_t planCloudListPtr)
{
	cell_name_ = boost::get<std::string>(parameters()["name"]);

	cluster_tolerance_ = boost::get<double>(parameters()["cluster_tolerance"]);
	min_cluster_size_ = static_cast<int>(
				boost::get<double>(parameters()["min_cluster_size"]));
	max_cluster_size_ = static_cast<int>(
				boost::get<double>(parameters()["max_cluster_size"]));

	euclidian_cluster_extractor_.setClusterTolerance (cluster_tolerance_);
	euclidian_cluster_extractor_.setMinClusterSize (min_cluster_size_);
	euclidian_cluster_extractor_.setMaxClusterSize (max_cluster_size_);

	for(std::size_t j = 0;j<planCloudListPtr->size(); ++j)
	{
		std::vector<pcl::PointIndices> cluster_indices_out;
		euclidian_cluster_extractor_.setInputCloud (
					planCloudListPtr->at(j).cloud());
		euclidian_cluster_extractor_.extract (cluster_indices_out);

		for (std::vector<pcl::PointIndices>::const_iterator it =
			 cluster_indices_out.begin (); it != cluster_indices_out.end (); ++it)
		{
			plan_cloud_ptr_ = boost::make_shared<PlanCloud > ();
			for (std::vector<int>::const_iterator pit = it->indices.begin ();
				pit != it->indices.end (); pit++)
				plan_cloud_ptr_->cloud()->points.push_back
						(planCloudListPtr->at(j).cloud()->points[
						 static_cast<unsigned long>(*pit)]);

			new_plan_cloud_list_ptr_->push_back(*plan_cloud_ptr_);
		}
	}

	return new_plan_cloud_list_ptr_;
}
