#ifndef FILECELL_H
#define FILECELL_H

#include "cell.h"

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

///This class implements the reading of a .pcl file and the creation of a PointCloud object from it
class FileCell : public Cell
{
public:
	FileCell();
	///The compute function is useless here (because signatures don't match)
	boost::shared_ptr<std::vector<PlanCloud> > compute(boost::shared_ptr<std::vector<PlanCloud> >);

	///This method implements the reading of a .pcl file and the creation of a PointCloud object from it
	boost::shared_ptr<std::vector<PlanCloud> > sync(std::string path, boost::shared_ptr<std::vector<PlanCloud> > planCloudListPtr);
};

#endif // FILECELL_H
