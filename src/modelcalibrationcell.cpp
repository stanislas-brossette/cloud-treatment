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
#include <pcl/search/kdtree.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>

#include "modelcalibrationcell.h"
#include "plancloud.h"
#include "typedefs.h"

#include "dirs.hh"

ModelCalibrationCell::ModelCalibrationCell():
	Cell("ModelCalibrationCell"),
	views_(),
	uniform_sampling_(),
	descr_est_()
{
	parameters()["number_of_neighbours_normal_estimation"] = 10;
	parameters()["keypoint_search_radius_scene"] = 0.1f;
	parameters()["descriptor_search_radius_scene"] = 0.1f;
	std::string cadModelFile = "chair.ply";

//	generateViewsFromCADModelFile(cadModelFile);
//	std::cout << std::endl << "Generated " << views_.size() << " views of " << cadModelFile << std::endl;


}

planCloudsPtr_t ModelCalibrationCell::compute(planCloudsPtr_t planCloudListPtr)
{
	number_of_neighbours_normal_estimation_ =
			static_cast<int>(parameters()["number_of_neighbours_normal_estimation"]);
	keypoint_search_radius_scene_ =
			static_cast<float>(parameters()["keypoint_search_radius_scene"]);
	descriptor_search_radius_scene_ =
			static_cast<float>(parameters()["descriptor_search_radius_scene"]);

	for(pointCloudPoints_t::size_type j = 0; j<planCloudListPtr->size(); ++j)
	{
		//
		// Compute normals of the scene
		//
		planCloudListPtr->at(j).normals() = computeNormals(
					planCloudListPtr->at(j).cloud());

		//
		// Downsample Cloud to Extract keypoints
		//
		planCloudListPtr->at(j).keyPoints() = computeKeypoints(
					planCloudListPtr->at(j).cloud(), keypoint_search_radius_scene_);

		std::cout << "Scene total points: " << planCloudListPtr->at(j).cloud()->size ()
				  << "; Selected Keypoints: " << planCloudListPtr->at(j).keyPoints()->size () << std::endl;

		//
		// Compute Descriptor for keypoints
		//

		planCloudListPtr->at(j).descriptors() = computeDescriptors(
					planCloudListPtr->at(j).cloud(),
					planCloudListPtr->at(j).normals(),
					planCloudListPtr->at(j).keyPoints(),
					descriptor_search_radius_scene_);


		std::cout << "Scene total points: " << planCloudListPtr->at(j).cloud()->size ()
				  << "; Selected Keypoints: " << planCloudListPtr->at(j).keyPoints()->size ()
				  << "; Descriptors: " << planCloudListPtr->at(j).descriptors()->size ()
				  << std::endl;

		//
		// Find Model-Scene Correspondences with KdTree
		//

	}

	return planCloudListPtr;
}

normalCloudPtr_t ModelCalibrationCell::computeNormals(
		const pointCloudPtr_t& point_cloud_ptr_)
{
	normalCloudPtr_t normals = boost::make_shared<normalCloud_t> ();
	pcl::search::Search<pcl::PointXYZ>::Ptr tree =
			boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >
			(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (point_cloud_ptr_);
	normal_estimator.setKSearch (number_of_neighbours_normal_estimation_);
	normal_estimator.compute (*normals);

	return normals;
}

pointCloudPtr_t ModelCalibrationCell::computeKeypoints(
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

descriptorCloudPtr_t ModelCalibrationCell::computeDescriptors(
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

void ModelCalibrationCell::generateViewsFromCADModelFile(std::string cadModelFile)
{
	boost::filesystem::path cadModelPath = findCADModelFile(cadModelFile);

	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New ();
	reader->SetFileName (cadModelPath.c_str());
	vtkSmartPointer < vtkPolyDataMapper > mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
	mapper->SetInputConnection (reader->GetOutputPort ());
	mapper->Update ();

	vtkSmartPointer< vtkPolyData > polydata = vtkSmartPointer< vtkPolyData > (mapper->GetInput ());

	pcl::apps::RenderViewsTesselatedSphere render_views;
	render_views.setResolution(100);

	render_views.addModelFromPolyData (polydata);

	render_views.generateViews ();
	render_views.getViews(views_);
}

boost::filesystem::path ModelCalibrationCell::findCADModelFile(std::string cadModelFile)
{
	namespace fs = boost::filesystem;

	fs::path cadModelPath (cadModelFile);
	fs::path cadModelPathInstall (CAD_MODEL_PATH);
	cadModelPathInstall /= cadModelFile;
	fs::path cadModelPathBuild (CAD_MODEL_BUILD_PATH);
	cadModelPathBuild /= cadModelFile;

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
					% cadModelFile).str () << std::endl;
		throw std::runtime_error("CAD model file not found");
	}

	return cadModelPath;
}


