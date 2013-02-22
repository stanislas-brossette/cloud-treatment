#ifndef APPLICATION_H
#define APPLICATION_H

#include <vector>

#include <boost/shared_ptr.hpp>

#include "plancloud.h"
#include "custompclvisualizor.h"
#include "typedefs.h"

/// \brief Simple application that contains the workflow
///
/// The list of operation that will be applied to the pointclouds is
/// implemented in the objects that this class instanciates

class Application
{
public:
	Application();
	virtual ~Application ()
	{
	}

	/// \brief this function instanciates all the algorithm object and
	/// then runs them on the pointClouds
	void Run();

private:
	CustomPCLVisualizor customPCLVisualizor;
};

#endif // APPLICATION_H
