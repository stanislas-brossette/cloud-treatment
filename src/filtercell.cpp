#include <iostream>
#include "filtercell.h"

FilterCell::FilterCell()
{
}

boost::shared_ptr<std::vector<PlanCloud> > FilterCell::compute(boost::shared_ptr<std::vector<PlanCloud> > planCloudListPtr)
{
	std::cout<<"Hello, that's filterCell here, I've been created :D"<< std::endl;
	return planCloudListPtr;
}
