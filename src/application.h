#ifndef APPLICATION_H
#define APPLICATION_H

#include <vector>

#include <boost/shared_ptr.hpp>

#include "plancloud.h"

///This class is a simple application that contains the workflow
class Application
{
public:
	Application();
	void Run();
	void display_all_clouds(boost::shared_ptr<std::vector<PlanCloud> >);
	void display_all_clouds_together(boost::shared_ptr<std::vector<PlanCloud> >);
	void display_all_coefficients(boost::shared_ptr<std::vector<PlanCloud> >);
};

#endif // APPLICATION_H
