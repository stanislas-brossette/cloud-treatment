#ifndef CADMODELTREATMENTCELL_H
#define CADMODELTREATMENTCELL_H

#include <boost/filesystem/path.hpp>

#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/shot_omp.h>

#include "cell.h"
#include "typedefs.h"

class CADModelTreatmentCell : public Cell
{
public:
	CADModelTreatmentCell();

	virtual ~CADModelTreatmentCell ()
	{
	}

	planCloudsPtr_t compute(planCloudsPtr_t);

private:
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
	void addToDataBase();

	std::vector< pointCloudPtr_t > views_;
	std::vector< Eigen::Matrix4f, Eigen::aligned_allocator< Eigen::Matrix4f > > views_poses_;
	std::vector< normalCloudPtr_t > views_normals_;
	std::vector< pointCloudPtr_t > views_keypoints_;
	std::vector< descriptorCloudPtr_t > views_descriptors_;

	pcl::UniformSampling<pcl::PointXYZ> uniform_sampling_;
	pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, DescriptorType> descr_est_;
	pcl::search::Search<pcl::PointXYZ>::Ptr tree_;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator_;

	std::string databaseName_;
	std::string objectName_;
	std::string cadModelFile_;
	int views_resolution_;
	int number_of_neighbours_normal_estimation_;
	float keypoint_search_radius_model_;
	float descriptor_search_radius_model_;
};

#endif // CADMODELTREATMENTCELL_H
