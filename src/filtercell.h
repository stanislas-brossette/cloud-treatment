#ifndef FILTERCELL_H
#define FILTERCELL_H

#include <vector>
#include <string>

#include <boost/shared_ptr.hpp>

#include "cell.h"

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

private:
	/// Shared pointer that contains the pointCloud object
	pointCloudPtr_t cloud_ptr_;
	/// Sizes of the voxel-grid
	float leafX_, leafY_, leafZ_;
};

#endif // FILTERCELL_H
