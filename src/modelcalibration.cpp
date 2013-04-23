#include <iostream>
#include <stdexcept>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "application.h"


int main(int argc, char** argv)
{
	Application app;
	std::string path;
	if (argc < 2)
	{
		path = "crapahut";
		std::cout<<"Without input argument, the default file will be used (" << path << ".pcd)"<<std::endl;
	}
	else
	{
		path = argv[1];
	}

	app = Application(path);

	app.createFromYaml("../share/yaml/modelcalibration.yaml");

	try
	{
		app.Run();
	}
	catch(std::exception &e)
	{
		std::cout<<e.what()<<std::endl;
	}
}


