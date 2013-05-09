#include <vector>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <yaml-cpp/yaml.h>

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
	views_poses_(),
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
	parameters()["databaseName"] = "testingDatabase";
	parameters()["objectName"] = "bunny";
	parameters()["cadModelFile"] = "bunny.ply";
	parameters()["views_resolution"] = 100;
	parameters()["number_of_neighbours_normal_estimation"] = 10;
	parameters()["keypoint_search_radius_model"] = 0.1f;
	parameters()["descriptor_search_radius_model"] = 0.1f;
}

planCloudsPtr_t CADModelTreatmentCell::compute(planCloudsPtr_t planCloudListPtr)
{
	cell_name_ = boost::get<std::string>(parameters()["name"]);
	databaseName_ = boost::get<std::string>(parameters()["databaseName"]);
	objectName_ = boost::get<std::string>(parameters()["objectName"]);
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

	boost::filesystem::path cadModelPath = findCADModelFile(cadModelFile_);

	generateViewsFromCADModelFile(cadModelPath);
	std::cout << std::endl << "Generated " << views_.size() <<
				 " views of " << cadModelFile_ << std::endl;

	views_poses_.resize(views_.size());
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

	addToDataBase();

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

void CADModelTreatmentCell::generateViewsFromCADModelFile(boost::filesystem::path cadModelPath)
{
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

boost::filesystem::path CADModelTreatmentCell::findCADModelFile(
		std::string cadModelFile_)
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

void CADModelTreatmentCell::addToDataBase()
{
	namespace fs = boost::filesystem;
	typedef boost::format frm;

	//Check for the database existence and create it if necessary
	fs::path databasePath ((frm("%1%/%2%")
							% RECOGNITION_DATABASE_PATH % databaseName_).str());
	if (!fs::is_directory(databasePath))
	{
		fs::create_directory(databasePath);
		std::cout << databaseName_ << " database has been created" << std::endl;
	}

	//Check for the yaml database file and create it if necessary
	fs::path dbYamlFilePath (
		(frm("%1%/%2%.db.yaml") % databasePath.string() % databaseName_).str());
	if (!fs::exists(dbYamlFilePath))
	{
		fs::ofstream dbYamlInitFile(dbYamlFilePath);
		std::cout << (frm("%1% \nhas been created\n") % dbYamlFilePath).str();
	}

	//Search in the document for the objectName and add it if necessary
	fs::ifstream dbYamlFile(dbYamlFilePath);
	if (!dbYamlFile.good ())
		throw std::runtime_error ("bad stream (database content reading)");
	YAML::Parser parser (dbYamlFile);

	YAML::Node doc;

	if (!parser.GetNextDocument (doc))
	{
		YAML::Emitter out;
		out << YAML::BeginSeq;
		out << objectName_;
		out << YAML::EndSeq;
		fs::ofstream dbYamlWriteFile(dbYamlFilePath);
		dbYamlWriteFile << out.c_str();
	}

	bool objectFound = false;
	for (YAML::Iterator it = doc.begin (); it != doc.end (); ++it)
	{
		std::string scalar;
		*it >> scalar;
		if(scalar == objectName_)
		{
			std::string errorMessage(boost::str(frm(
				"ERROR: The object %1% already exists in the database %2%!")
									  % objectName_ % databaseName_));
			throw std::runtime_error(errorMessage);

			objectFound = true;
			break;
		}
	}

	if(objectFound)
	{
		return;
	}

	std::cout<< "Adding " << objectName_ << " to " << databaseName_;
	YAML::Emitter outDatabase;
	outDatabase << YAML::BeginSeq;
	for (YAML::Iterator it = doc.begin (); it != doc.end (); ++it)
	{
		std::string scalar;
		*it >> scalar;
		outDatabase << scalar;
	}
	outDatabase << objectName_;
	outDatabase << YAML::EndSeq;
	fs::ofstream dbYamlWriteFile(dbYamlFilePath);
	dbYamlWriteFile << outDatabase.c_str();

	//Add the folders for the object in the DB
	fs::path objectPath(
				(frm("%1%/%2%") % databasePath.string() % objectName_).str());
	fs::create_directory(objectPath);

	fs::path viewsPath((frm("%1%/views") % objectPath.string()).str());
	fs::path normalsPath((frm("%1%/normals") % objectPath.string()).str());
	fs::path descriptorsPath((frm("%1%/descriptors") % objectPath.string()).str());
	fs::path keypointsPath((frm("%1%/keypoints") % objectPath.string()).str());
	fs::path posesPath((frm("%1%/poses") % objectPath.string()).str());
	fs::create_directory(viewsPath);
	fs::create_directory(normalsPath);
	fs::create_directory(descriptorsPath);
	fs::create_directory(keypointsPath);
	fs::create_directory(posesPath);

	//Add the object yaml file
	YAML::Emitter outObject;
	outObject << YAML::BeginMap;
	outObject << YAML::Key << "parameters";
	outObject << YAML::Value << YAML::BeginMap;
		outObject << YAML::Key << "views_resolution";
		outObject << YAML::Value << views_resolution_;
		outObject << YAML::Key << "number_of_neighbours_normal_estimation";
		outObject << YAML::Value << number_of_neighbours_normal_estimation_;
		outObject << YAML::Key << "keypoint_search_radius_model";
		outObject << YAML::Value << keypoint_search_radius_model_;
		outObject << YAML::Key << "descriptor_search_radius_model";
		outObject << YAML::Value << descriptor_search_radius_model_;
		outObject << YAML::EndMap;
	outObject << YAML::Key << "cadModel";
	outObject << YAML::Value << (findCADModelFile(cadModelFile_)).string();
	outObject << YAML::Key << "views";
	outObject << YAML::Value << viewsPath.string();
	outObject << YAML::Key << "poses";
	outObject << YAML::Value << posesPath.string();
	outObject << YAML::Key << "normals";
	outObject << YAML::Value << normalsPath.string();
	outObject << YAML::Key << "keypoints";
	outObject << YAML::Value << keypointsPath.string();
	outObject << YAML::Key << "descriptors";
	outObject << YAML::Value << descriptorsPath.string();
	outObject << YAML::EndMap;

	fs::path objYamlFilePath (
		(frm("%1%/%2%.obj.yaml") % objectPath.string() % objectName_ ).str());
	if (!fs::exists(objYamlFilePath))
	{
		fs::ofstream objYamlFile(objYamlFilePath);
		objYamlFile << outObject.c_str();
	}

	//Filling the folders

	for (size_t i = 0; i < views_.size(); ++i)
	{
		fs::path viewFilePath(
					(frm("%1%/view%2%.pcd") % viewsPath.string() % i).str());
		pcl::io::savePCDFile(viewFilePath.string(), *(views_[i]));

		fs::path poseFilePath(
					(frm("%1%/pose%2%.txt") % posesPath.string() % i).str());
		fs::ofstream poseFile(poseFilePath);
		poseFile << views_poses_[i];

		fs::path normalFilePath(
					(frm("%1%/normals%2%.pcd") % normalsPath.string() % i).str());
		pcl::io::savePCDFile(normalFilePath.string(), *(views_normals_[i]));

		fs::path keypointFilePath(
					(frm("%1%/keypoints%2%.pcd") % keypointsPath.string() % i).str());
		pcl::io::savePCDFile(keypointFilePath.string(), *(views_keypoints_[i]));

		fs::path descriptorFilePath(
					(frm("%1%/descriptors%2%.pcd") % descriptorsPath.string() % i).str());
		pcl::io::savePCDFile(descriptorFilePath.string(), *(views_descriptors_[i]));
	}
	std::cout << ": Done\n";
}





































