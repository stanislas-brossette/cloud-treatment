#ifndef APPLICATION_H
#define APPLICATION_H

#include <vector>

#include <boost/shared_ptr.hpp>

#include "plancloud.h"
#include "custompclvisualizor.h"
#include "typedefs.h"
#include "cell.h"

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
	void Run();

	std::vector < boost::shared_ptr < Cell > >& cells()
	{
		return cells_;
	}

	const std::vector < boost::shared_ptr < Cell > >& cells() const
	{
		return cells_;
	}

private:
	CustomPCLVisualizor customPCLVisualizor;
	std::string pcd_file_name;
	std::vector < boost::shared_ptr < Cell > > cells_;
};

#endif // APPLICATION_H
