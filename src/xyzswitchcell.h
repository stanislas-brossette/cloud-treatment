#ifndef XYXSWITCHCELL_H
#define XYXSWITCHCELL_H

#include "cell.h"

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

class XYZSwitchCell : public Cell
{
public:
	XYZSwitchCell();
	boost::shared_ptr<std::vector<PlanCloud> > compute(boost::shared_ptr<std::vector<PlanCloud> >);
};

#endif // XYXSWITCHCELL_H
