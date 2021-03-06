#ifndef MODELCALIBRATIONCELL_H
#define MODELCALIBRATIONCELL_H

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <pcl/correspondence.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/shot_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "cell.h"
#include "typedefs.h"

/// \brief Calibrate the position of a 3D model object on a given point-cloud
/// cluster. First the model data are extracted from a database, the the best
/// view of the model is selected based on the correspondences between its
/// descriptors and the clouds descriptors and finally the optimal position
/// of the model is estimated

class ModelCalibrationCell : public Cell
{
public:
	ModelCalibrationCell();

	virtual ~ModelCalibrationCell ()
	{
	}

	planCloudsPtr_t compute(planCloudsPtr_t);

private:

	void loadDatabaseInfo();
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
	boost::filesystem::path cad_model_path_;
	boost::filesystem::path views_path_;
	boost::filesystem::path poses_path_;
	boost::filesystem::path normals_path_;
	boost::filesystem::path keypoints_path_;
	boost::filesystem::path descriptors_path_;


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


	template <typename T>
	void loadCloudsFromDirectory (
			const boost::filesystem::path& path,
			std::vector<typename pcl::PointCloud<T>::Ptr>& container)
	{
		namespace fs = boost::filesystem;
		if (fs::exists(path))
		{
			fs::directory_iterator end ;
			std::vector<std::string> accumulator;
			for( fs::directory_iterator iter(path) ; iter != end ; ++iter ) {
				if ( !fs::is_directory( *iter ) )
				{
					accumulator.push_back((iter->path()).string());
				}
			}
			std::sort(accumulator.begin(), accumulator.end());
			std::vector<std::string>::iterator iter;
			for (iter = accumulator.begin(); iter != accumulator.end(); ++iter)
			{
				typename pcl::PointCloud<T>::Ptr view = boost::make_shared<pcl::PointCloud<T> > ();
				if(pcl::io::loadPCDFile<T> (*iter, *view) != -1)
					container.push_back(view);
				else
					throw std::runtime_error(
							(boost::format("Failed to load file %1%") % *iter).str());
			}
		}
	}

	template <typename T>
	void loadOneCloudFromDirectory (
			const boost::filesystem::path& path,
			const std::string& type,
			typename pcl::PointCloud<T>::Ptr& container,
			unsigned long index)
	{
		namespace fs = boost::filesystem;
		fs::path cloudPath((boost::format("%s/%s%05i.pcd") % path.string() % type % index).str());
		if(pcl::io::loadPCDFile<T> (cloudPath.string(), *container) != -1)
			;
		else
			throw std::runtime_error(
					(boost::format("Failed to load file %1%") % cloudPath).str());
	}

	void loadPosesFromDirectory (
			const boost::filesystem::path& posesPath,
			std::vector< Eigen::Matrix4f, Eigen::aligned_allocator< Eigen::Matrix4f > >& container);
	Eigen::Matrix4f loadOnePoseFromDirectory (
			const boost::filesystem::path& posesPath, unsigned long index);
};

#endif // MODELCALIBRATIONCELL_H
