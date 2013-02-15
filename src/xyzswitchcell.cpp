#include "xyzswitchcell.h"

XYZSwitchCell::XYZSwitchCell()
{
}


boost::shared_ptr<std::vector<PlanCloud> > XYZSwitchCell::compute(boost::shared_ptr<std::vector<PlanCloud> > planCloudListPtr)
{
	for(int j=0; j<planCloudListPtr->size(); j++)
	{
		float x, y, z;
		for(int i = 0; i<planCloudListPtr->at(j).cloud()->points.size (); i++)
		{
			if(!isnan(planCloudListPtr->at(j).cloud()->points[i].x))
			{
				x = planCloudListPtr->at(j).cloud()->points[i].z;
				y = -planCloudListPtr->at(j).cloud()->points[i].x;
				z = -planCloudListPtr->at(j).cloud()->points[i].y;

				planCloudListPtr->at(j).cloud()->points[i].x = x;
				planCloudListPtr->at(j).cloud()->points[i].y = y;
				planCloudListPtr->at(j).cloud()->points[i].z = z;
			}
		}
	}
	return planCloudListPtr;
}
