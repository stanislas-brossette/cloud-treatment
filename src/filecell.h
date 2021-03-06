#ifndef FILECELL_H
#define FILECELL_H

#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include "cell.h"
#include "filecell.h"
#include "typedefs.h"


/// \brief Implements the reading of a .pcl file and the creation of a
/// PointCloud object from it
class FileCell : public Cell
{
public:
	FileCell();
	virtual ~FileCell ()
	{
	}

	/// The compute function is useless here
	/// (because signatures don't match)
	planCloudsPtr_t compute(planCloudsPtr_t);

	/// \brief This method implements the reading of a .pcl file and the
	/// creation of a PointCloud object from it.
	///
	/// The pointCloud is stored in a PlanCloud that is stored in a list.
	/// That list is the object that we are going to pass from object
	/// to object to make it go through all the algorithm we want.
	planCloudsPtr_t sync(std::string path, planCloudsPtr_t planCloudListPtr);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sync_color(std::string path);
};

#endif // FILECELL_H
