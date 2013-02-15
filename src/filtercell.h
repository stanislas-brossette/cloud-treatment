#ifndef FILTERCELL_H
#define FILTERCELL_H

#include "cell.h"

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/PointCloud2.h>

///This class implements a voxel grid filter that reduces the number of points in the treated cloud
class FilterCell : public Cell
{
public:
	FilterCell();
	///Constructor overload that allows to choose the size of the voxel-grid's leafs.
	FilterCell(float leafX,float  leafY,float leafZ);
	boost::shared_ptr<std::vector<PlanCloud> > compute(boost::shared_ptr<std::vector<PlanCloud> >);

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_;
	sensor_msgs::PointCloud2::Ptr cloud2_ptr_;
	float leafX_, leafY_, leafZ_;
};

#endif // FILTERCELL_H
