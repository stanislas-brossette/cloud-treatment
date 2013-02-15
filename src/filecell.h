#ifndef FILECELL_H
#define FILECELL_H

#include "cell.h"

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

class FileCell : public Cell
{
public:
	FileCell();
	boost::shared_ptr<std::vector<PlanCloud> > compute(boost::shared_ptr<std::vector<PlanCloud> >);
	boost::shared_ptr<std::vector<PlanCloud> > sync(std::string path, boost::shared_ptr<std::vector<PlanCloud> > planCloudListPtr);
};

#endif // FILECELL_H
