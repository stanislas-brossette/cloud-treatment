
#include <vector>
#include <string>
#include <fstream>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <yaml-cpp/yaml.h>

#include "application.h"
#include "factory.h"
#include "filecell.h"
#include "plancloud.h"
#include "typedefs.h"
#include "visualizer.h"


Application::Application():
	pcd_file_name(""),
	cells_(),
	factory_()
{
}

Application::Application(std::string path):
	pcd_file_name(path),
	cells_(),
	factory_()
{
}

void Application::Run()
{
	planCloudsPtr_t planCloudListPtr = boost::make_shared < planClouds_t > ();

	FileCell fileCell = FileCell();

	Visualizer visualizer = Visualizer();

	fileCell.sync(pcd_file_name, planCloudListPtr);

	for(std::size_t i = 0; i < cells_.size(); ++i)
	{
		std::cout << cells_[i]->cell_name() << std::endl;
		planCloudListPtr = cells_[i]->compute(planCloudListPtr);
		std::cout << planCloudListPtr << std::endl;
		if(i ==0)
		{
			visualizer.add_xyz_clouds(planCloudListPtr);
		}
	}

	visualizer.add_convex_clouds(planCloudListPtr);
	visualizer.display_all();

//	std::cout<<Verbose(1)<<planCloudListPtr;

}

void Application::createFromYaml(const std::string& yamlFilename)
{
	std::ifstream fin(yamlFilename.c_str());
	if (!fin.good ())
		throw std::runtime_error ("bad stream");
	YAML::Parser parser (fin);

	YAML::Node doc;

	if (!parser.GetNextDocument (doc))
		throw std::runtime_error ("empty document");

	std::string version;
	doc["version"] >> version;
	std::cout << "version = " << version << std::endl;



	for (YAML::Iterator it = doc["pipeline"].begin ();
			 it != doc["pipeline"].end (); ++it)
	{
		std::string cellType;
		(*it)["type"] >> cellType;
		std::cout << cellType << "\n";
		boost::shared_ptr< Cell > cell = factory_.create(cellType);
		cells_.push_back(cell);
	}

//	const YAML::Node& pipeline = doc["pipeline"];
//	for(unsigned i=0;i<pipeline.size();i++) {
//		std::string cellType;
//		pipeline[i]["type"] >> cellType;
//		std::cout << cellType << "\n";
//	}
}

