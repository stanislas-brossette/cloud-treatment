#ifndef MODELCALIBRATIONCELL_H
#define MODELCALIBRATIONCELL_H

#include <boost/filesystem/path.hpp>

#include "cell.h"
#include "typedefs.h"

class ModelCalibrationCell : public Cell
{
public:
	ModelCalibrationCell();

	virtual ~ModelCalibrationCell ()
	{
	}

	planCloudsPtr_t compute(planCloudsPtr_t);

private:

	void generateViewsFromCADModelFile (std::string cadModelFile);
	boost::filesystem::path findCADModelFile(std::string cadModelFile);

	std::vector< pointCloudPtr_t > views_;

};

#endif // MODELCALIBRATIONCELL_H
