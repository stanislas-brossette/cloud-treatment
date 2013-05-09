#ifndef MODELCALIBRATIONCELL_H
#define MODELCALIBRATIONCELL_H

#include <boost/filesystem/path.hpp>

#include <pcl/correspondence.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/shot_omp.h>

#include "cell.h"
#include "typedefs.h"

class ModelCalibrationCell : public Cell
{
public:
	ModelCalibrationCell();

	virtual ~ModelCalibrationCell ()
	{
	}

	planCloudsPtr_t compute(planCloudsPtr_t);

private:

	void loadModelFromDatabase();
	boost::filesystem::path findCADModelFile(std::string cadModelFile);
	normalCloudPtr_t computeNormals(const pointCloudPtr_t& pointCloudPtr);
	pointCloudPtr_t computeKeypoints(const pointCloudPtr_t& pointCloudPtr,
									 const float& search_radius);
	descriptorCloudPtr_t computeDescriptors(
			const pointCloudPtr_t& pointCloudPtr,
			const normalCloudPtr_t& normalCloudPtr,
			const pointCloudPtr_t& keypointCloudPtr,
			const float& search_radius);

	std::vector< pointCloudPtr_t > views_;
	std::vector< Eigen::Matrix4f, Eigen::aligned_allocator< Eigen::Matrix4f > > views_poses_;
	std::vector< normalCloudPtr_t > views_normals_;
	std::vector< pointCloudPtr_t > views_keypoints_;
	std::vector< descriptorCloudPtr_t > views_descriptors_;
	std::vector< pcl::CorrespondencesPtr > model_scene_corrs_;

	pcl::UniformSampling<pcl::PointXYZ> uniform_sampling_;
	pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, DescriptorType> descr_est_;
	pcl::search::Search<pcl::PointXYZ>::Ptr tree_;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator_;

	std::string database_;
	std::string model_;
	int number_of_neighbours_normal_estimation_;
	float keypoint_search_radius_;
	float descriptor_search_radius_;
	float correspondence_grouping_size_;
	int correspondence_grouping_threshhold_;

	//parameters of the model
	int views_resolution_model_;
	int number_of_neighbours_normal_estimation_model_;
	float keypoint_search_radius_model_;
	float descriptor_search_radius_model_;
};

#endif // MODELCALIBRATIONCELL_H
