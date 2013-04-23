#include <iostream>
#include <stdexcept>
#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>

#include "application.h"


namespace po = boost::program_options;

int main(int argc, char** argv)
{

	std::string pointCloudFile;
	std::string pipelineFile;

	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("point_cloud,C", po::value< std::string >(), "input point-cloud file")
		("pipeline,P", po::value< std::string >(), "input pipeline file")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if (vm.count("help")) {
		std::cout << desc << "\n";
		return 1;
	}

	if (vm.count("point_cloud"))
	{
		pointCloudFile = vm["point_cloud"].as< std::string >();
		std::cout << "Point cloud input file was set to "
			 << vm["point_cloud"].as< std::string >() << ".\n";
	}
	else
	{
		pointCloudFile = "../datafiles/TableClimbing.pcd";
		std::cout << "Point cloud input file was set its to default value: "
				<< pointCloudFile << "\n";
	}

	if (vm.count("pipeline"))
	{
		pipelineFile = vm["pipeline"].as< std::string >();
		std::cout << "Pipeline input file was set to "
			 << vm["pipeline"].as< std::string >() << ".\n";
	}
	else
	{
		pipelineFile = "../share/yaml/cloudtreatment.yaml";
		std::cout << "Pipeline input file was set to its default value: "
				<< pipelineFile << "\n";
	}


	Application app;

	app = Application(pointCloudFile);

	app.createFromYaml(pipelineFile);

	try
	{
		app.Run();
	}
	catch(std::exception &e)
	{
		std::cout<<e.what()<<std::endl;
	}
}


