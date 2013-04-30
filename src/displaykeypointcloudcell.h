#ifndef DISPLAYKEYPOINTCLOUDCELL_H
#define DISPLAYKEYPOINTCLOUDCELL_H

#include <boost/shared_ptr.hpp>

#include "application.h"
#include "cell.h"

class Application;

class DisplayKeypointCloudCell : public Cell
{
public:
	/// \name Constructors
	/// \{
	DisplayKeypointCloudCell(Application& application_ref);
	/// \}

	virtual ~DisplayKeypointCloudCell ()
	{
	}

	planCloudsPtr_t compute(planCloudsPtr_t);

private:
	Application& application_ref_;
};
#endif // DISPLAYKEYPOINTCLOUDCELL_H
