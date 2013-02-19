#ifndef APPLICATION_H
#define APPLICATION_H

#include <vector>

#include <boost/shared_ptr.hpp>

#include "plancloud.h"
#include "typedefs.h"

///Simple application that contains the workflow
class Application
{
public:
	Application();
	void Run();
	void display_all_clouds(planCloudsPtr_t);
	void display_all_clouds_together(planCloudsPtr_t);
	void display_all_coefficients(planCloudsPtr_t);
};

#endif // APPLICATION_H
