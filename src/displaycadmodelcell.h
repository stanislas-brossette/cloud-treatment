#ifndef DISPLAYCADMODELCELL_H
#define DISPLAYCADMODELCELL_H

#include <boost/shared_ptr.hpp>

#include "application.h"
#include "cell.h"

class Application;

class DisplayCADModelCell : public Cell
{
public:
	/// \name Constructors
	/// \{
	DisplayCADModelCell(Application& application_ref);
	/// \}

	virtual ~DisplayCADModelCell ()
	{
	}

	planCloudsPtr_t compute(planCloudsPtr_t);

private:
	Application& application_ref_;
};

#endif // DISPLAYCADMODELCELL_H
