#ifndef XYXSWITCHCELL_H
#define XYXSWITCHCELL_H

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include "cell.h"

///This class switches the XYZ coordinates in a PointCloud to match with the XYZ that are imposed by AMELIF
class XYZSwitchCell : public Cell
{
public:
	XYZSwitchCell();
	boost::shared_ptr<std::vector<PlanCloud> > compute(boost::shared_ptr<std::vector<PlanCloud> >);

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_;
};

#endif // XYXSWITCHCELL_H
