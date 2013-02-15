#ifndef FILTERCELL_H
#define FILTERCELL_H

#include "cell.h"

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

class FilterCell : public Cell
{
public:
	FilterCell();
	boost::shared_ptr<std::vector<PlanCloud> > compute(boost::shared_ptr<std::vector<PlanCloud> >);
};

#endif // FILTERCELL_H
