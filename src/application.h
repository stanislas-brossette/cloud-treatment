#ifndef APPLICATION_H
#define APPLICATION_H

#include <vector>

#include <boost/shared_ptr.hpp>

#include "plancloud.h"
#include "custompclvisualizor.h"
#include "typedefs.h"

///Simple application that contains the workflow
class Application
{
public:
	Application();
	void Run();
	void display_all_coefficients(planCloudsPtr_t);

private:
	CustomPCLVisualizor customPCLVisualizor;
};

#endif // APPLICATION_H
