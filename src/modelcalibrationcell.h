#ifndef MODELCALIBRATIONCELL_H
#define MODELCALIBRATIONCELL_H

#include <boost/filesystem/path.hpp>

#include <pcl/keypoints/uniform_sampling.h>

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
									 float searchRadius);

	std::vector< pointCloudPtr_t > views_;
	pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;

	int number_of_neighbours_normal_estimation_;
	float keypoint_search_radius_scene_;
};

#endif // MODELCALIBRATIONCELL_H
