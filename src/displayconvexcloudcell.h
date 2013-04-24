#ifndef DISPLAYCONVEXCLOUDCELL_H
#define DISPLAYCONVEXCLOUDCELL_H

#include <boost/shared_ptr.hpp>

#include "application.h"
#include "cell.h"

class Application;

class DisplayConvexCloudCell : public Cell
{
public:
	/// \name Constructors
	/// \{
	DisplayConvexCloudCell(Application& application_ref);
	/// \}

	virtual ~DisplayConvexCloudCell ()
	{
	}

	planCloudsPtr_t compute(planCloudsPtr_t);

private:
	Application& application_ref_;
};

#endif // DISPLAYCONVEXCLOUDCELL_H
