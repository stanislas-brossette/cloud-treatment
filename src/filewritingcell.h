#ifndef FILEWRITINGCELL_H
#define FILEWRITINGCELL_H

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include "typedefs.h"
#include "cell.h"
#include "filewritingcell.h"

class FileWritingCell : public Cell
{
public:
	FileWritingCell();
	///The compute function is useless here (because signatures don't match)
	planCloudsPtr_t compute(planCloudsPtr_t);

	///This method implements the writing of a .surf file that represents the contours
	///of the plans that have been found in the pointcloud in a way that Amelif can understand
	void write_files(std::string folderPath, planCloudsPtr_t planCloudListPtr, std::string bodyName = "point_cloud_body");

private:
	std::string write_surf_string(planCloudPtr_t planCloudPtr, std::string name);
};

#endif // FILEWRITINGCELL_H
