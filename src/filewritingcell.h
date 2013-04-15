#ifndef FILEWRITINGCELL_H
#define FILEWRITINGCELL_H

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include "typedefs.h"
#include "cell.h"
#include "filewritingcell.h"

/// \brief Class that writes a file that represents the cloud in a way
/// that Amelif can read
///
/// So far this class is meant to write a file that represents a 2D
/// polygon, which can be obtained from a cloud that has been through
/// the HullConvexCell algorithm.
/// \see HullConvexCell
/// In the future this class could be improved to write different types
/// of files

class FileWritingCell : public Cell
{
public:
	FileWritingCell();
	virtual ~FileWritingCell ()
	{
	}
	/// The compute function is useless here
	/// (because signatures don't match)
	planCloudsPtr_t compute(planCloudsPtr_t);

	/// This method implements the writing of a .surf file that represents
	/// the polygons that are contained in a list of pointCloud in a way
	/// that Amelif can understand, a ".surf" file.
	///
	/// \pre all the planClouds of the list must have been through the
	/// PlanExtractionCell algorithm
	planCloudsPtr_t write_files(std::string folderPath, planCloudsPtr_t planCloudListPtr, std::string bodyName = "point_cloud_body");


	void write_cloud_files(std::string folderPath, std::string filename, planCloudsPtr_t planCloudListPtr);

private:
	/// Method that gets called by \see write_files , it writes the part
	/// of the file that represents a single planCloud.
	std::string write_surf_string(planCloudPtr_t planCloudPtr, std::string name);
};

#endif // FILEWRITINGCELL_H
