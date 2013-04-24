#include <iostream>
#include <stdexcept>
#include <string>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include "application.h"

#include "dirs.hh"

namespace po = boost::program_options;

int main(int argc, char** argv)
{
	std::string pointCloudFile;
	std::string pipelineFile;

	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("point-cloud,c", po::value<std::string>(&pointCloudFile),
		 "input point-cloud file")
		("pipeline,p", po::value<std::string>(&pipelineFile),
		 "input pipeline file")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if (vm.count("help")) {
		std::cout << desc << "\n";
		return 0;
	}

	if (!vm.count("point-cloud"))
	{
	  std::cout << "point cloud file is required" << std::endl;
	  return 1;
	}

	if (!vm.count("pipeline"))
	{
	  std::cout << "pipeline file is required" << std::endl;
	  return 1;
	}

	namespace fs = boost::filesystem;

	fs::path pointCloudPath (pointCloudFile);
	fs::path pointCloudPathInstall (POINT_CLOUD_PATH);
	pointCloudPathInstall /= pointCloudFile;
	fs::path pointCloudPathBuild (POINT_CLOUD_BUILD_PATH);
	pointCloudPathBuild /= pointCloudFile;

	if (fs::is_regular_file (pointCloudPath))
	  {}
	else if (fs::is_regular_file (pointCloudPathInstall))
	  pointCloudPath = pointCloudPathInstall;
	else if (fs::is_regular_file (pointCloudPathBuild))
	  pointCloudPath = pointCloudPathBuild;
	else
	  {
	    std::cout
	      << (boost::format ("point cloud file \"%1%\" does not exist")
		  % pointCloudFile).str ()
	      << std::endl;
	    return 1;
	  }

	Application app (pointCloudPath.native ());

	fs::path pipelinePath (pipelineFile);
	fs::path pipelinePathInstall (PIPELINE_PATH);
	pipelinePathInstall /= pipelineFile;
	fs::path pipelinePathBuild (PIPELINE_BUILD_PATH);
	pipelinePathBuild /= pipelineFile;

	if (fs::is_regular_file (pipelinePath))
	  app.createFromYaml (pipelinePath.native ());
	else if (fs::is_regular_file (pipelinePathInstall))
	  app.createFromYaml (pipelinePathInstall.native ());
	else if (fs::is_regular_file (pipelinePathBuild))
	  app.createFromYaml (pipelinePathBuild.native ());
	else
	  {
	    std::cout
	      << (boost::format ("pipeline file \"%1%\" does not exist")
		  % pipelineFile).str ()
	      << std::endl;
	    return 1;
	  }

	try
	{
		app.run();
	}
	catch(std::exception &e)
	{
		std::cout<<e.what()<<std::endl;
	}
}


