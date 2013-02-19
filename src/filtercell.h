#ifndef FILTERCELL_H
#define FILTERCELL_H

#include "cell.h"

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/PointCloud2.h>

///Implements a voxel grid filter that reduces the number of points in the treated cloud
class FilterCell : public Cell
{
public:
	FilterCell();
	///Constructor overload that allows to choose the size of the voxel-grid's leafs.
	FilterCell(float leafX,float  leafY,float leafZ);
	planCloudsPtr_t compute(planCloudsPtr_t);

private:
	pointCloudPtr_t cloud_ptr_;
	pointCloud2Ptr_t cloud2_ptr_;
	float leafX_, leafY_, leafZ_;
};

#endif // FILTERCELL_H
