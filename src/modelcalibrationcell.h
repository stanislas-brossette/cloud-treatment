#ifndef MODELCALIBRATIONCELL_H
#define MODELCALIBRATIONCELL_H

#include "cell.h"

class ModelCalibrationCell : public Cell
{
public:
	ModelCalibrationCell();

	virtual ~ModelCalibrationCell ()
	{
	}

	planCloudsPtr_t compute(planCloudsPtr_t);
private:

};

#endif // MODELCALIBRATIONCELL_H
