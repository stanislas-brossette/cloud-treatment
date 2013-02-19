#ifndef CELL_H
# define CELL_H

# include <vector>
# include <boost/shared_ptr.hpp>

# include "typedefs.h"
# include "plancloud.h"

//The parent from which each algorithm class inherits
class Cell
{
public:
	Cell();

	///The algorithm method
	virtual planCloudsPtr_t compute(planCloudsPtr_t) = 0;
};

#endif // CELL_H
