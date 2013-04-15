#ifndef FILTERCELL_H
#define FILTERCELL_H

#include "cell.h"

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/PointCloud2.h>

/// \brief Implements a voxel grid filter that reduces the number of points
/// in the treated cloud
class FilterCell : public Cell
{
public:
	/// \name Constructors
	/// \{
	FilterCell();
	/// Constructor overload that allows to choose the size of the
	/// voxel-grid's leafs.
	FilterCell(float leafX,float  leafY,float leafZ);
	/// \}

	virtual ~FilterCell ()
	{
	}

	/// Filters the point cloud in order to reduce the number of points it
	/// contains whilst avoiding the lost of information
	planCloudsPtr_t compute(planCloudsPtr_t);

	/// Optional filter that eliminates the points that are too far from the camera
	/// Those points are not precise enough to be used
	planCloudsPtr_t computePassThrough(planCloudsPtr_t, float maxDistance);
	planCloudsPtr_t computePassThrough(planCloudsPtr_t, std::string axis, float min, float max);


private:
	/// Shared pointer that contains the pointCloud object
	pointCloudPtr_t cloud_ptr_;
	/// Shared pointer that contains the pointCloud2 object
	//pointCloud2Ptr_t cloud2_ptr_;
	/// Sizes of the voxel-grid
	float leafX_, leafY_, leafZ_;
};

#endif // FILTERCELL_H
