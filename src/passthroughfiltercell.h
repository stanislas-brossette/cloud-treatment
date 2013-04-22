#ifndef PASSTHROUGHFILTERCELL_H
#define PASSTHROUGHFILTERCELL_H

#include <vector>
#include <string>

#include <boost/shared_ptr.hpp>

#include "cell.h"

class PassThroughFilterCell : public Cell
{
public:
	PassThroughFilterCell();

	virtual ~PassThroughFilterCell ()
	{
	}

	planCloudsPtr_t compute(planCloudsPtr_t);


private:
	pointCloudPtr_t cloud_ptr_;
	std::string axis;
	float min_;
	float max_;
};

#endif // PASSTHROUGHFILTERCELL_H
