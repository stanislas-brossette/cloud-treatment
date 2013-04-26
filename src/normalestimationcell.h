#ifndef NORMALESTIMATIONCELL_H
#define NORMALESTIMATIONCELL_H

#include "cell.h"
#include "typedefs.h"

class NormalEstimationCell : public Cell
{
public:
	NormalEstimationCell();

	virtual ~NormalEstimationCell ()
	{
	}

	planCloudsPtr_t compute(planCloudsPtr_t);

private:
	int number_of_neighbours_normal_estimation_;
	pointCloudPtr_t point_cloud_ptr_;
};

#endif // NORMALESTIMATIONCELL_H
