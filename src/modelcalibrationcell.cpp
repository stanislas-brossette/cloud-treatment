#include "modelcalibrationcell.h"
#include "plancloud.h"

ModelCalibrationCell::ModelCalibrationCell():
	Cell("ModelCalibrationCell")
{
}

planCloudsPtr_t ModelCalibrationCell::compute(planCloudsPtr_t planCloudListPtr)
{
	std::cout << cell_name_ << "::compute called" << std::endl;
	return planCloudListPtr;
}
