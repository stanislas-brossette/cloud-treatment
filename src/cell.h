#ifndef CELL_H
#define CELL_H

#include <vector>
#include <boost/shared_ptr.hpp>

#include "plancloud.h"

///This class is the parent from which each algorithm class inherits
class Cell
{
public:
	Cell();

	///Tha algorithm method
	virtual boost::shared_ptr<std::vector<PlanCloud> > compute(boost::shared_ptr<std::vector<PlanCloud> >) = 0;
};

#endif // CELL_H
