#include "application.h"

#include <iostream>
#include <stdexcept>



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

	try
	{
		app.Run();
	}
	catch(std::exception &e)
	{
		std::cout<<e.what()<<std::endl;
	}
}



