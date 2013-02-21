#ifndef FILECELL_H
#define FILECELL_H

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include "typedefs.h"
#include "filecell.h"
#include "cell.h"

///Implements the reading of a .pcl file and the creation of a PointCloud object from it
class FileCell : public Cell
{
public:
	FileCell();
	///The compute function is useless here (because signatures don't match)
	planCloudsPtr_t compute(planCloudsPtr_t);

	///This method implements the reading of a .pcl file and the creation of a PointCloud object from it
	planCloudsPtr_t sync(std::string path, planCloudsPtr_t planCloudListPtr);
};

#endif // FILECELL_H
