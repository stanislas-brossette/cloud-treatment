#include "organizedsegmentationcell.h"
#include "plancloud.h"
#include "typedefs.h"

OrganizedSegmentationCell::OrganizedSegmentationCell():
	Cell()
{
}

planCloudsPtr_t OrganizedSegmentationCell::compute(planCloudsPtr_t planCloudListPtr)
{
	return planCloudListPtr;
}
