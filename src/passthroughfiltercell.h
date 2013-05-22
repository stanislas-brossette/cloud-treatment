#ifndef PASSTHROUGHFILTERCELL_H
#define PASSTHROUGHFILTERCELL_H

#include <string>
#include <vector>

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
	std::string axis_;
	float min_;
	float max_;
	int keep_organized_;
};

#endif // PASSTHROUGHFILTERCELL_H
