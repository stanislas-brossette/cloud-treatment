# include <math.h>

# include "typedefs.h"
# include "orientationcell.h"

OrientationCell::OrientationCell():
	Cell("OrientationCell")
{
}

planCloudsPtr_t OrientationCell::compute(planCloudsPtr_t planCloudListPtr)
{

	for (pointCloudSize_t i = 0 ; i < planCloudListPtr->size(); ++i)
	{
		if(!planCloudListPtr->at(i).origin())
			planCloudListPtr->at(i).find_origin();
		if(!planCloudListPtr->at(i).frame())
			planCloudListPtr->at(i).find_frame();
		planCloudListPtr->at(i).set_trigo_order();
		planCloudListPtr->at(i).project_points_on_frame();
	}
	std::cout<<"PlanCloudList before rotation"<<std::endl<<planCloudListPtr<<std::endl<<std::endl;
	int groundID = this->findGroundID(planCloudListPtr);//ID of the surface that represents the ground;
	std::cout << "groundID = " << groundID << std::endl;
	Eigen::Matrix3d rotation;
	double theta = 25; //in degree
	theta = theta*M_PI/180;
	rotation << cos(theta),	0,	sin(theta),
				0,			1,	0,
				-sin(theta),0,	cos(theta);
	Eigen::Vector3d center;
	center << 0, 0, 0;

	for(pointCloudSize_t i = 0 ; i < planCloudListPtr->size(); ++i)
	{
		this->rotateSurf(rotation, center, planCloudListPtr->at(i));
	}
	return planCloudListPtr;
}

int OrientationCell::findGroundID(planCloudsPtr_t planCloudListPtr)
{
	//In a first time, to simplify, we considere that the ground is the surface
	//with the most vertical Normal vector
	double maxZ = -1;
	int groundID = -1;

	for (pointCloudSize_t i = 0 ; i < planCloudListPtr->size(); ++i)
	{
		if (planCloudListPtr->at(i).N().z() > maxZ)
		{
			maxZ = planCloudListPtr->at(i).N().z();
			groundID = static_cast<int>(i);
		}
	}
	return groundID;
}

void OrientationCell::rotate(Eigen::Matrix3d& rotation, Eigen::Vector3d& myVector)
{
	myVector = rotation*myVector;
}

void OrientationCell::rotateSurf(Eigen::Matrix3d& rotation, Eigen::Vector3d& center, PlanCloud& planCloud)
{
	rotate(rotation, planCloud.T());
	rotate(rotation, planCloud.B());
	rotate(rotation, planCloud.N());
	*(planCloud.frame()) << planCloud.T(), planCloud.B(), planCloud.N();

	Eigen::Vector3d OgOi(planCloud.origin()->x() - center.x(),
						 planCloud.origin()->y() - center.y(),
						 planCloud.origin()->z() - center.z());
	OgOi = rotation*OgOi;
	*(planCloud.origin()) << OgOi.x(), OgOi.y(), OgOi.z();
}

