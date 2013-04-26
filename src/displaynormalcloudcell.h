#ifndef DISPLAYNORMALCLOUDCELL_H
#define DISPLAYNORMALCLOUDCELL_H

#include <boost/shared_ptr.hpp>

#include "application.h"
#include "cell.h"

class Application;

class DisplayNormalCloudCell : public Cell
{
public:
	/// \name Constructors
	/// \{
	DisplayNormalCloudCell(Application& application_ref);
	/// \}

	virtual ~DisplayNormalCloudCell ()
	{
	}

	planCloudsPtr_t compute(planCloudsPtr_t);

private:
	Application& application_ref_;
};

#endif // DISPLAYNORMALCLOUDCELL_H
