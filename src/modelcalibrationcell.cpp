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

#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>

#include "modelcalibrationcell.h"
#include "plancloud.h"
#include "typedefs.h"

#include "dirs.hh"

ModelCalibrationCell::ModelCalibrationCell():
	Cell("ModelCalibrationCell"),
	point_cloud_ptr_(boost::make_shared<pointCloud_t > ())
{
	parameters()["number_of_neighbours_normal_estimation"] = 10;
	std::string cadModelFile = "chair.ply";
//	generateViewsFromCADModelFile(cadModelFile);
	std::cout << std::endl << "Generated " << views_.size() << " views of " << cadModelFile << std::endl;

}

planCloudsPtr_t ModelCalibrationCell::compute(planCloudsPtr_t planCloudListPtr)
{
	number_of_neighbours_normal_estimation_ =
			static_cast<int>(parameters()["number_of_neighbours_normal_estimation"]);

	for(pointCloudPoints_t::size_type j = 0; j<planCloudListPtr->size(); ++j)
	{
		point_cloud_ptr_ = planCloudListPtr->at(j).cloud();
		pcl::search::Search<pcl::PointXYZ>::Ptr tree =
				boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >
				(new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
		normal_estimator.setSearchMethod (tree);
		normal_estimator.setInputCloud (point_cloud_ptr_);
		normal_estimator.setKSearch (number_of_neighbours_normal_estimation_);
		normal_estimator.compute (*(planCloudListPtr->at(j).normals()));
	}

	return planCloudListPtr;
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


