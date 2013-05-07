#include <fstream>
#include <string>
#include <vector>

#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>
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
	factory_(*this),
	visualizer_()
{
}

Application::Application(std::string path):
	pcd_file_name(path),
	cells_(),
	factory_(*this),
	visualizer_()
{
}

void Application::run()
{
	planCloudsPtr_t planCloudListPtr = boost::make_shared < planClouds_t > ();

	FileCell fileCell = FileCell();

	fileCell.sync(pcd_file_name, planCloudListPtr);
	for(std::size_t i = 0; i < cells_.size(); ++i)
	{
		std::cout << cells_[i]->parameters()["name"] << std::endl;
		planCloudListPtr = cells_[i]->compute(planCloudListPtr);
	}

	//Triggers the display only if a display cell is found
	for(std::size_t i = 0; i < cells_.size(); ++i)
	{
		if(cells_[i]->parameters().find("name") == cells_[i]->parameters().end())
		{
			boost::format fmt ("cell number %1% doesn't have a name!");
			fmt % i;
			throw std::runtime_error(fmt.str());
		}
		std::string cellName = boost::get<std::string>(cells_[i]->parameters()["name"]);
		if(cellName == "DisplayConvexCloudCell"
				|| cellName == "DisplayKeypointCloudCell"
				|| cellName == "DisplayNormalCloudCell"
				|| cellName == "DisplayXYZCloudCell")
		{
			visualizer_.display_all();
			break;
		}
	}
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
				paramIt.first() >> key;

				try
				{
					double value;
					paramIt.second() >> value;
					if(cell->parameters().find(key) != cell->parameters().end())
						cell->parameters()[key] = value;
					else
						throw(std::runtime_error("wrong parameter key in yaml"));
				}
				catch(const YAML::InvalidScalar&)
				{
					std::string value;
					paramIt.second() >> value;
					if(cell->parameters().find(key) != cell->parameters().end())
						cell->parameters()[key] = value;
					else
						throw(std::runtime_error("wrong parameter key in yaml"));
				}
			}
		}

		cells_.push_back(cell);
	}
}

void Application::setCellParameter(const std::string& cellName,
								   const std::string& parameter,
								   const std::string& value)
{
	for(std::size_t i = 0; i < cells_.size(); ++i)
	{
		if(cells_[i]->parameters().find("name") == cells_[i]->parameters().end())
		{
			boost::format fmt ("cell number %1% doesn't have a name!");
			fmt % i;
		}
		if(boost::get<std::string>(cells_[i]->parameters()["name"]) !=
				cellName)
			continue;
		try
		{
			cells_[i]->parameters()[parameter] =
					boost::lexical_cast<double>(value);
		}
		catch(boost::bad_lexical_cast &)
		{
			cells_[i]->parameters()[parameter] = value;
		}
		return;
	}

	boost::format fmt ("parameter %1%.%2% not found!");
	fmt % cellName % parameter;

	throw std::runtime_error(fmt.str());
}

