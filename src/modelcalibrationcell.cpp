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
#include <pcl/common/transforms.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/recognition/cg/geometric_consistency.h>
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
	Cell(),
	views_(),
	views_normals_(),
	views_keypoints_(),
	views_descriptors_(),
	model_scene_corrs_(),
	uniform_sampling_(),
	descr_est_(),
	tree_(boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >
		  (new pcl::search::KdTree<pcl::PointXYZ>)),
	normal_estimator_()

{
	parameters()["name"] = "ModelCalibrationCell";
	parameters()["database"] = "";
	parameters()["model"] = "";
	parameters()["number_of_neighbours_normal_estimation"] = 10;
	parameters()["keypoint_search_radius"] = 0.1f;
	parameters()["descriptor_search_radius"] = 0.1f;
	parameters()["correspondence_grouping_size"] = 0.05f;
	parameters()["correspondence_grouping_threshhold"] = 5;
}

planCloudsPtr_t ModelCalibrationCell::compute(planCloudsPtr_t planCloudListPtr)
{
	cell_name_ = boost::get<std::string>(parameters()["name"]);

	database_ = boost::get<std::string>(parameters()["database"]);
	model_ = boost::get<std::string>(parameters()["model"]);
	number_of_neighbours_normal_estimation_ =
			static_cast<int>(boost::get<double>(
				parameters()["number_of_neighbours_normal_estimation"]));
	keypoint_search_radius_ =
			static_cast<float>(boost::get<double>(
				parameters()["keypoint_search_radius"]));
	descriptor_search_radius_ =
			static_cast<float>(boost::get<double>(
				parameters()["descriptor_search_radius"]));
	correspondence_grouping_size_ =
			static_cast<float>(boost::get<double>(
				parameters()["correspondence_grouping_size"]));
	correspondence_grouping_threshhold_ =
			static_cast<int>(boost::get<double>(
				parameters()["correspondence_grouping_threshhold"]));

	if(database_ == "")
		throw std::runtime_error("Database name is mandatory");
	if(model_ == "")
		throw std::runtime_error("Model name is mandatory");

	//Chech the database to extract the parameters of the model and the paths
	//to the different folders containing the views, descriptors, normals,
	//cad model and keypoints
	loadDatabaseInfo();

	//Loads all the descriptor clouds of the object to calibrate
	this->loadCloudsFromDirectory<DescriptorType>(descriptors_path_, views_descriptors_);

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
					planCloudListPtr->at(j).cloud(), keypoint_search_radius_);

		//
		// Compute Descriptor for keypoints
		//
		planCloudListPtr->at(j).descriptors() = computeDescriptors(
					planCloudListPtr->at(j).cloud(),
					planCloudListPtr->at(j).normals(),
					planCloudListPtr->at(j).keyPoints(),
					descriptor_search_radius_);


		std::cout<<"Scene total points: " << planCloudListPtr->at(j).cloud()->size() << " points\n"
				<<planCloudListPtr->at(j).normals()->size() << " normals\n"
				<<planCloudListPtr->at(j).keyPoints()->size() << " keypoints\n"
				<<planCloudListPtr->at(j).descriptors()->size() << "descriptors \n\n";

		//
		// Find Model-Scene Correspondences with KdTree
		//

		std::size_t best_view_index = 0;
		std::size_t max_correspondences = 0;

		model_scene_corrs_.resize(views_descriptors_.size());
		for(std::size_t k = 0; k < views_descriptors_.size(); ++k)
		{
			model_scene_corrs_[k] = boost::make_shared <pcl::Correspondences> ();
			pcl::KdTreeFLANN<DescriptorType> match_search;
			match_search.setInputCloud (views_descriptors_[k]);

			//  For each scene keypoint descriptor, find nearest neighbor into the model
			// keypoints descriptor cloud and add it to the correspondences vector.
			for (size_t i = 0; i < planCloudListPtr->at(j).descriptors()->size (); ++i)
			{
				std::vector<int> neigh_indices (1);
				std::vector<float> neigh_sqr_dists (1);
				if (!pcl_isfinite (planCloudListPtr->at(j).descriptors()->at (i).descriptor[0])) //skipping NaNs
				{
					continue;
				}
				int found_neighs = match_search.nearestKSearch (
							planCloudListPtr->at(j).descriptors()->at (i), 1,
							neigh_indices, neigh_sqr_dists);
				//  add match only if the squared descriptor distance is less than 0.25
				// (SHOT descriptor distances are between 0 and 1 by design)
				if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
				{
					pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i),
											  neigh_sqr_dists[0]);
					model_scene_corrs_[k]->push_back (corr);
				}
			}
//			std::cout << "Correspondences with view_[" << k << "] found: " << model_scene_corrs_[k]->size () << std::endl;

			// For now, we considere that the best view is the one that has the highest
			// number of correspondences
			// TO BE IMPROVED
			if(model_scene_corrs_[k]->size () > max_correspondences)
			{
				max_correspondences = model_scene_corrs_[k]->size ();
				best_view_index = k;
			}
		}
		std::cout << "Best view: " << best_view_index << " with " <<
					 max_correspondences << " correspondences.\n";

		//
		//  Actual Clustering
		//
		std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >
				rototranslations;
		std::vector<pcl::Correspondences> clustered_corrs;

		pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ>
				gc_clusterer;
		gc_clusterer.setGCSize (correspondence_grouping_size_);
		gc_clusterer.setGCThreshold (correspondence_grouping_threshhold_);

		pointCloudPtr_t bestKeypoints = boost::make_shared<pointCloud_t> ();
		this->loadOneCloudFromDirectory<pcl::PointXYZ>( keypoints_path_, "keypoints",
								   bestKeypoints, best_view_index);
		gc_clusterer.setInputCloud (bestKeypoints);
		gc_clusterer.setSceneCloud (planCloudListPtr->at(j).keyPoints());
		gc_clusterer.setModelSceneCorrespondences (model_scene_corrs_[best_view_index]);

		//gc_clusterer.cluster (clustered_corrs);
		gc_clusterer.recognize (rototranslations, clustered_corrs);

		std::cout << "Model instances found: " << rototranslations.size ()
				  << std::endl;
		for (size_t i = 0; i < rototranslations.size (); ++i)
		{
			std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
			std::cout << "        Correspondences belonging to this instance: "
					  << clustered_corrs[i].size () << std::endl;

			// Print the rotation matrix and translation vector
			Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
			Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

			printf ("\n");
			printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
			printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
			printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
			printf ("\n");
			printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
		}



		pointCloudPtr_t bestView = boost::make_shared<pointCloud_t> ();
		this->loadOneCloudFromDirectory<pcl::PointXYZ>( views_path_, "view",
								   bestView, best_view_index);

		//Adding the cad model to the planCloudList
		for (size_t i = 0; i < rototranslations.size (); ++i)
		{
			std::stringstream ss_model;
			ss_model << "model_instance" << i;

			vtkSmartPointer< vtkMatrix4x4 > rototransBestViewToScene =
					vtkSmartPointer< vtkMatrix4x4 >::New();
			pcl::visualization::PCLVisualizer::convertToVtkMatrix(
						rototranslations[i], rototransBestViewToScene);

			vtkSmartPointer< vtkMatrix4x4 > rototransPlyToBestView =
					vtkSmartPointer< vtkMatrix4x4 >::New();
			pcl::visualization::PCLVisualizer::convertToVtkMatrix(
						loadOnePoseFromDirectory(poses_path_, best_view_index),
						rototransPlyToBestView);

			vtkSmartPointer<vtkTransform> transformPly =
					vtkSmartPointer<vtkTransform>::New();
			transformPly->SetMatrix(rototransBestViewToScene);
			transformPly->Concatenate(rototransPlyToBestView);

			planCloudListPtr->at(j).cad_models().push_back(
						boost::make_shared<std::pair<std::string, vtkSmartPointer<vtkTransform> > >(
							std::make_pair(
								cad_model_path_.string(),transformPly)));
		}
	}

	return planCloudListPtr;
}

normalCloudPtr_t ModelCalibrationCell::computeNormals(
		const pointCloudPtr_t& point_cloud_ptr_)
{
	normalCloudPtr_t normals = boost::make_shared<normalCloud_t> ();
	normal_estimator_.setSearchMethod (tree_);
	normal_estimator_.setInputCloud (point_cloud_ptr_);
	normal_estimator_.setKSearch (number_of_neighbours_normal_estimation_);
	normal_estimator_.compute (*normals);

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

void ModelCalibrationCell::loadDatabaseInfo()
{
	namespace fs = boost::filesystem;
	typedef boost::format frm;
	fs::path databasePath ((frm("%1%/%2%")
							% RECOGNITION_DATABASE_PATH % database_).str());
	if (!fs::is_directory(databasePath))
	{
		std::string errorMessage(boost::str(frm(
			"The database \n%1%\ncan't be found!")
								  % databasePath));
		throw std::runtime_error(errorMessage);
	}

	// Check the existence of the database yaml file
	fs::path dbYamlFilePath (
		(frm("%1%/%2%.db.yaml") % databasePath.string() % database_).str());
	if (!fs::exists(dbYamlFilePath))
	{
		std::string errorMessage(boost::str(frm(
			"The file \n%1%\ncan't be found!")
								  % dbYamlFilePath));
		throw std::runtime_error(errorMessage);
	}

	//Search in the document for the objectName and add it if necessary
	fs::ifstream dbYamlFile(dbYamlFilePath);
	if (!dbYamlFile.good ())
		throw std::runtime_error ("bad stream (database content reading)");

	YAML::Parser parserDB (dbYamlFile);

	YAML::Node docDB;

	if (!parserDB.GetNextDocument (docDB))
	{
		std::string errorMessage(boost::str(frm(
			"The file \n%1%\nis empty!")
								  % dbYamlFilePath));
		throw std::runtime_error(errorMessage);
	}

	bool objectFound = false;
	for (YAML::Iterator it = docDB.begin (); it != docDB.end (); ++it)
	{
		std::string scalar;
		*it >> scalar;
		if(scalar == model_)
		{
			objectFound = true;
			break;
		}
	}

	if(!objectFound)
	{
		std::string errorMessage(boost::str(frm(
			"The model %1% is not in the database %2%!") % model_ % database_));
		throw std::runtime_error(errorMessage);
		return;
	}

	fs::path modelPath((frm("%1%/%2%") % databasePath.string() % model_).str());
	fs::path modelYamlFilePath (
		(frm("%1%/%2%.obj.yaml") % modelPath.string() % model_).str());
	// The model is part of the database, now the loading starts
	fs::ifstream modelYamlFile(modelYamlFilePath);
	if (!modelYamlFile.good ())
		throw std::runtime_error ("bad stream (model content reading)");

	YAML::Parser parser (modelYamlFile);

	YAML::Node doc;
	if (!parser.GetNextDocument (doc))
	{
		throw std::runtime_error("The description file of this object is empty!");
	}

	for (YAML::Iterator it = doc.begin ();
			 it != doc.end (); ++it)
	{
		std::string key, value;
		it.first() >> key;
		if ( key != "parameters")
			it.second() >> value;

		if (key == "parameters")
		{
			for (YAML::Iterator itParam = doc["parameters"].begin ();
					 itParam != doc["parameters"].end (); ++itParam)
			{
				std::string keyP;
				double valueP;
				itParam.first() >> keyP;
				itParam.second() >> valueP;
				if (keyP == "views_resolution")
					views_resolution_model_ = static_cast<int>(valueP);
				else if (keyP == "number_of_neighbours_normal_estimation")
					number_of_neighbours_normal_estimation_model_ = static_cast<int>(valueP);
				else if (keyP == "keypoint_search_radius_model")
					keypoint_search_radius_model_ = static_cast<float>(valueP);
				else if (keyP == "descriptor_search_radius_model")
					descriptor_search_radius_model_ = static_cast<float>(valueP);
				else
					throw std::runtime_error("non-recognized parameter in model yaml file");
			}
		}
		else if (key == "cad_model")
			cad_model_path_ = fs::path(value);
		else if (key == "views")
			views_path_ = fs::path(value);
		else if (key == "poses")
			poses_path_ = fs::path(value);
		else if (key == "normals")
			normals_path_ = fs::path(value);
		else if (key == "keypoints")
			keypoints_path_ = fs::path(value);
		else if (key == "descriptors")
			descriptors_path_ = fs::path(value);
		else
			throw std::runtime_error("non-recognized field in model yaml file");
	}

//	std::cout<<
//				"views_resolution_model_ = "<< views_resolution_model_ << std::endl <<
//				"number_of_neighbours_normal_estimation_model_ = "<< number_of_neighbours_normal_estimation_model_ << std::endl <<
//				"keypoint_search_radius_model_ = "<< keypoint_search_radius_model_ << std::endl <<
//				"descriptor_search_radius_model_ = "<< descriptor_search_radius_model_ << std::endl <<
//				"cadModelPath = "<< cad_model_path_ << std::endl <<
//				"viewsPath = "<< views_path_ << std::endl <<
//				"posesPath = "<< poses_path_ << std::endl <<
//				"normalsPath = "<< normals_path_ << std::endl <<
//				"keypointsPath = "<< keypoints_path_ << std::endl <<
//				"descriptorsPath = "<< descriptors_path_ << std::endl;
}

void ModelCalibrationCell::loadPosesFromDirectory (
		const boost::filesystem::path& posesPath,
		std::vector< Eigen::Matrix4f, Eigen::aligned_allocator< Eigen::Matrix4f > >& container)
{
	namespace fs = boost::filesystem;
	if (fs::exists(posesPath))
	{
		fs::directory_iterator end ;
		std::vector<std::string> accumulator;
		for( fs::directory_iterator iter(posesPath) ; iter != end ; ++iter ) {
			if ( !fs::is_directory( *iter ) )
			{
				accumulator.push_back((iter->path()).string());
			}
		}
		std::sort(accumulator.begin(), accumulator.end());
		std::vector<std::string>::iterator iter;
		for (iter = accumulator.begin(); iter != accumulator.end(); ++iter)
		{
			fs::ifstream f(*iter);
			Eigen::Matrix4f pose = Eigen::Matrix4f();
			for(std::size_t i=0; i<4; ++i)
			{
				for(std::size_t j=0; j<4; ++j)
				{
					float q;
					f >> q;
					pose(static_cast<long>(i),static_cast<long>(j))=q;
				}
			}
			container.push_back(pose);
		}
	}
}

Eigen::Matrix4f ModelCalibrationCell::loadOnePoseFromDirectory (
		const boost::filesystem::path& posesPath, unsigned long index)
{
	namespace fs = boost::filesystem;
	fs::path cloudPath((boost::format("%s/pose%05i.txt") % posesPath.string() % index).str());
	fs::ifstream f(cloudPath.string());
	Eigen::Matrix4f pose = Eigen::Matrix4f();
	for(std::size_t i=0; i<4; ++i)
	{
		for(std::size_t j=0; j<4; ++j)
		{
			float q;
			f >> q;
			pose(static_cast<long>(i),static_cast<long>(j))=q;
		}
	}
	return pose;
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


