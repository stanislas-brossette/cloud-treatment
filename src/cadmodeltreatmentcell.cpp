#include <vector>
#include <string>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <eigen3/Eigen/Dense>

#include <pcl/apps/render_views_tesselated_sphere.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/search/kdtree.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "cadmodeltreatmentcell.h"

#include "dirs.hh"

CADModelTreatmentCell::CADModelTreatmentCell():
	Cell(),
	views_(),
	views_normals_(),
	views_keypoints_(),
	views_descriptors_(),
	uniform_sampling_(),
	descr_est_(),
	tree_(boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >
		  (new pcl::search::KdTree<pcl::PointXYZ>)),
	normal_estimator_()

{
	parameters()["name"] = "CADModelTreatmentCell";
	parameters()["cadModelFile"] = "bunny.ply";
	parameters()["views_resolution"] = 100;
	parameters()["number_of_neighbours_normal_estimation"] = 10;
	parameters()["keypoint_search_radius_model"] = 0.1f;
	parameters()["descriptor_search_radius_model"] = 0.1f;
}

planCloudsPtr_t CADModelTreatmentCell::compute(planCloudsPtr_t planCloudListPtr)
{
	cell_name_ = boost::get<std::string>(parameters()["name"]);
	cadModelFile_ = boost::get<std::string>(parameters()["cadModelFile"]);
	views_resolution_ =
			static_cast<int>(boost::get<double>(
				parameters()["views_resolution"]));
	number_of_neighbours_normal_estimation_ =
			static_cast<int>(boost::get<double>(
				parameters()["number_of_neighbours_normal_estimation"]));
	keypoint_search_radius_model_ =
			static_cast<float>(boost::get<double>(
				parameters()["keypoint_search_radius_model"]));
	descriptor_search_radius_model_ =
			static_cast<float>(boost::get<double>(
				parameters()["descriptor_search_radius_model"]));

	generateViewsFromCADModelFile(cadModelFile_);
	std::cout << std::endl << "Generated " << views_.size() <<
				 " views of " << cadModelFile_ << std::endl;

	views_normals_.resize(views_.size());
	views_keypoints_.resize(views_.size());
	views_descriptors_.resize(views_.size());

	for(std::size_t i = 0; i < views_.size(); ++i)
	{
		views_normals_[i] = computeNormals(views_[i]);
		views_keypoints_[i] = computeKeypoints(views_[i],
											   keypoint_search_radius_model_);
		views_descriptors_[i] = computeDescriptors(
					views_[i],
					views_normals_[i],
					views_keypoints_[i],
					descriptor_search_radius_model_);
		std::cout<<"views_["<<i<<"] contains " << views_[i]->size() << " points\n"
				<<views_normals_[i]->size() << " normals\n"
				<<views_keypoints_[i]->size() << " keypoints\n"
				<<views_descriptors_[i]->size() << "descriptors \n\n";
	}
	return planCloudListPtr;
}

normalCloudPtr_t CADModelTreatmentCell::computeNormals(
		const pointCloudPtr_t& point_cloud_ptr_)
{
	normalCloudPtr_t normals = boost::make_shared<normalCloud_t> ();
	normal_estimator_.setSearchMethod (tree_);
	normal_estimator_.setInputCloud (point_cloud_ptr_);
	normal_estimator_.setKSearch (number_of_neighbours_normal_estimation_);
	normal_estimator_.compute (*normals);

	return normals;
}

pointCloudPtr_t CADModelTreatmentCell::computeKeypoints(
		const pointCloudPtr_t& pointCloudPtr, const float& search_radius)
{
	pcl::PointCloud<int> sampledIndices;
	pointCloudPtr_t keypointCloudPtr = boost::make_shared<pointCloud_t> ();

	uniform_sampling_.setInputCloud (pointCloudPtr);
	uniform_sampling_.setRadiusSearch (search_radius);
	uniform_sampling_.compute (sampledIndices);
	pcl::copyPointCloud (*pointCloudPtr, sampledIndices.points, *keypointCloudPtr);
	return keypointCloudPtr;
}

descriptorCloudPtr_t CADModelTreatmentCell::computeDescriptors(
		const pointCloudPtr_t& pointCloudPtr,
		const normalCloudPtr_t& normalCloudPtr,
		const pointCloudPtr_t& keypointCloudPtr,
		const float& search_radius)
{
	descriptorCloudPtr_t descriptorCloudPtr = boost::make_shared<descriptorCloud_t> ();
	descr_est_.setRadiusSearch (search_radius);
	descr_est_.setInputCloud (keypointCloudPtr);
	descr_est_.setInputNormals (normalCloudPtr);
	descr_est_.setSearchSurface (pointCloudPtr);
	descr_est_.compute (*descriptorCloudPtr);
	return descriptorCloudPtr;
}

void CADModelTreatmentCell::generateViewsFromCADModelFile(std::string cadModelFile_)
{
	boost::filesystem::path cadModelPath = findCADModelFile(cadModelFile_);

	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New ();
	reader->SetFileName (cadModelPath.c_str());
	vtkSmartPointer < vtkPolyDataMapper > mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
	mapper->SetInputConnection (reader->GetOutputPort ());
	mapper->Update ();

	vtkSmartPointer< vtkPolyData > polydata = vtkSmartPointer< vtkPolyData > (mapper->GetInput ());

	pcl::apps::RenderViewsTesselatedSphere render_views;
	render_views.setResolution(views_resolution_);

	render_views.addModelFromPolyData (polydata);

	render_views.generateViews ();
	render_views.getViews(views_);
	render_views.getPoses(views_poses_);
}

boost::filesystem::path CADModelTreatmentCell::findCADModelFile(std::string cadModelFile_)
{
	namespace fs = boost::filesystem;

	fs::path cadModelPath (cadModelFile_);
	fs::path cadModelPathInstall (CAD_MODEL_PATH);
	cadModelPathInstall /= cadModelFile_;
	fs::path cadModelPathBuild (CAD_MODEL_BUILD_PATH);
	cadModelPathBuild /= cadModelFile_;

	if (fs::is_regular_file (cadModelPath))
	{}
	else if (fs::is_regular_file (cadModelPathInstall))
		cadModelPath = cadModelPathInstall;
	else if (fs::is_regular_file (cadModelPathBuild))
		cadModelPath = cadModelPathBuild;
	else
	{
		std::cout
				<< (boost::format ("CAD model file \"%1%\" does not exist")
					% cadModelFile_).str () << std::endl;
		throw std::runtime_error("CAD model file not found");
	}

	return cadModelPath;
}


