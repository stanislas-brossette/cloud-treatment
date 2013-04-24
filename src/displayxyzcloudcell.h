#ifndef DISPLAYXYZCLOUDCELL_H
#define DISPLAYXYZCLOUDCELL_H

#include <boost/shared_ptr.hpp>

#include "application.h"
#include "cell.h"

class Application;

class DisplayXYZCloudCell : public Cell
{
public:
	/// \name Constructors
	/// \{
	DisplayXYZCloudCell(Application& application_ref);
	/// \}

	virtual ~DisplayXYZCloudCell ()
	{
	}

	planCloudsPtr_t compute(planCloudsPtr_t);

private:
	Application& application_ref_;
};

#endif // DISPLAYXYZCLOUDCELL_H
