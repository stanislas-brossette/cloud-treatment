#ifndef APPLICATION_H
#define APPLICATION_H

#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/variant.hpp>

#include "cell.h"
#include "factory.h"
#include "plancloud.h"
#include "typedefs.h"
#include "visualizer.h"


/// \brief Simple application that contains the workflow
///
/// The list of operation that will be applied to the pointclouds is
/// implemented in the objects that this class instanciates

class Application
{
public:
	Application();
	Application(std::string);
	virtual ~Application ()
	{
	}

	/// \brief this function instanciates all the algorithm object and
	/// then runs them on the pointClouds
	void run();

	std::vector < boost::shared_ptr < Cell > >& cells()
	{
		return cells_;
	}

	const std::vector < boost::shared_ptr < Cell > >& cells() const
	{
		return cells_;
	}

	Visualizer& visualizer()
	{
		return visualizer_;
	}

	const Visualizer& visualizer() const
	{
		return visualizer_;
	}

	// Parses a yaml file to setup the pipeline and its parameters
	void createFromYaml(const std::string& yamlFilename);

	// Set parameters from the command line, overwriting the yaml parameters
	// Syntax: cell_param=value
	void setCellParameter(const std::string& cellName,
						  const std::string& parameter,
						  const std::string& value);

private:
	std::string pcd_file_name;
	std::vector < boost::shared_ptr < Cell > > cells_;
	Factory factory_;
	Visualizer visualizer_;
};

#endif // APPLICATION_H
