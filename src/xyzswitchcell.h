#ifndef XYXSWITCHCELL_H
# define XYXSWITCHCELL_H

# include <string>
# include <vector>

# include <boost/shared_ptr.hpp>

# include "cell.h"
# include "typedefs.h"

/// Switches the XYZ coordinates in a PointCloud to match
/// with the XYZ that are imposed by AMELIF
class XYZSwitchCell : public Cell
{
public:
	XYZSwitchCell();
	virtual ~XYZSwitchCell ()
	{
	}

	/// Switches the XYZ coordinates in a PointCloud to match
	/// with the XYZ that are imposed by AMELIF
	planCloudsPtr_t compute(planCloudsPtr_t);
private:
	pointCloudPtr_t cloud_ptr_;
};

#endif // XYXSWITCHCELL_H
