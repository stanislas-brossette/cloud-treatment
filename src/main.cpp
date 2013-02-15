#include "application.h"

#include <iostream>
#include <stdexcept>



int main(int argc, char** argv)
{
	Application app;
	try
	{
		app.Run();
	}
	catch(std::exception &e)
	{
		std::cout<<e.what()<<std::endl;
	}
}


