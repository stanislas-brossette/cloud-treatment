#include <iostream>
#include <stdexcept>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "application.h"
#include "filecell.h"
#include "xyzswitchcell.h"
#include "filtercell.h"
#include "regiongrowingsegmentationcell.h"
#include "planextractioncell.h"
#include "planprojectioncell.h"
#include "hullconvexcell.h"
#include "filewritingcell.h"
#include "orientationcell.h"
#include "plancloud.h"
#include "typedefs.h"
#include "passthroughfiltercell.h"
#include "visualizer.h"


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

	app.cells().push_back(boost::make_shared<PassThroughFilterCell>());
	app.cells().push_back(boost::make_shared<FilterCell>());
	app.cells().push_back(boost::make_shared<RegionGrowingSegmentationCell>());
//	app.cells().push_back(boost::make_shared<XYZSwitchCell>());
	app.cells().push_back(boost::make_shared<PlanExtractionCell>());
	app.cells().push_back(boost::make_shared<PlanProjectionCell>());
	app.cells().push_back(boost::make_shared<HullConvexCell>());
//	app.cells().push_back(boost::make_shared<OrientationCell>());

	try
	{
		app.Run();
	}
	catch(std::exception &e)
	{
		std::cout<<e.what()<<std::endl;
	}
}


