#include <vector>
#include <string>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>

#include <eigen3/Eigen/Dense>

#include <pcl/apps/render_views_tesselated_sphere.h>
#include <pcl/io/pcd_io.h>

#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>

#include "modelcalibrationcell.h"
#include "plancloud.h"
#include "typedefs.h"

#include "dirs.hh"

ModelCalibrationCell::ModelCalibrationCell():
	Cell("ModelCalibrationCell")
{
	std::string cadModelFile = "bunny.ply";

	generateViewsFromCADModelFile(cadModelFile);
	std::cout << std::endl << "Generated " << views_.size() << " views of " << cadModelFile << std::endl;
//	for(std::size_t i = 0; i < views_.size(); ++i)
//	{
//		std::cout << "views_ " << i << ": " << views_[i]->points.size() << " points;" << std::endl;
//	}
}

planCloudsPtr_t ModelCalibrationCell::compute(planCloudsPtr_t planCloudListPtr)
{
	std::cout << cell_name_ << "::compute called" << std::endl;
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

