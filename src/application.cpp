
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
	visualizer.add_xyz_clouds(planCloudListPtr);

	for(std::size_t i = 0; i < cells_.size(); ++i)
	{
		std::cout << cells_[i]->cell_name() << std::endl;
		planCloudListPtr = cells_[i]->compute(planCloudListPtr);
		std::cout << planCloudListPtr << std::endl;
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
		boost::shared_ptr< Cell > cell = factory_.create(cellType);

		if(it->FindValue("parameters") != 0)
		{
			const YAML::Node& parameters = (*it)["parameters"];
			for(YAML::Iterator paramIt = parameters.begin();
				paramIt != parameters.end(); ++paramIt)
			{
				std::string key;
				double value;
				paramIt.first() >> key;
				paramIt.second() >> value;
				if(cell->parameters().find(key) != cell->parameters().end())
					cell->parameters()[key] = value;
				else
					throw(std::runtime_error("wrong parameter key in yaml"));
			}
		}

		cells_.push_back(cell);
	}
}

