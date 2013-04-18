
#include <vector>
#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "application.h"
#include "filecell.h"
#include "plancloud.h"
#include "typedefs.h"
#include "visualizer.h"


Application::Application()
{
}

Application::Application(std::string path)
{
	pcd_file_name = path;
}

void Application::Run()
{
	planCloudsPtr_t planCloudListPtr = boost::make_shared < planClouds_t > ();

	FileCell fileCell = FileCell();

	Visualizer visualizer = Visualizer();

	fileCell.sync(pcd_file_name, planCloudListPtr);

	for(std::size_t i = 0; i < cells_.size(); ++i)
	{
		std::cout << cells_[i]->cell_name() << std::endl;
		planCloudListPtr = cells_[i]->compute(planCloudListPtr);
		std::cout << planCloudListPtr << std::endl;
		if(i ==0)
		{
			visualizer.add_xyz_clouds(planCloudListPtr);
		}
	}

	visualizer.add_convex_clouds(planCloudListPtr);
	visualizer.display_all();

//	std::cout<<Verbose(1)<<planCloudListPtr;

}
