#ifndef MODELCALIBRATIONCELL_H
#define MODELCALIBRATIONCELL_H

#include <boost/filesystem/path.hpp>

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

	// Generates pointclouds as if they were taken from cameras all around the
	// CAD model
	void generateViewsFromCADModelFile (std::string cadModelFile);
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
	pcl::UniformSampling<pcl::PointXYZ> uniform_sampling_;
	pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, DescriptorType> descr_est_;

	int number_of_neighbours_normal_estimation_;
	float keypoint_search_radius_scene_;
	float descriptor_search_radius_scene_;
};

#endif // MODELCALIBRATIONCELL_H
