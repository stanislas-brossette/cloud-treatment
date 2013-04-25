#ifndef ORIENTATIONCELL_H
#define ORIENTATIONCELL_H

# include "cell.h"
# include "typedefs.h"

/// \brief Implements a method to re-orientate the clouds
class OrientationCell : public Cell
{
public:
	OrientationCell();
	virtual ~OrientationCell ()
	{
	}
	/// Re-orientate each of the clouds in the list
	/// \attention For now, the points of the cloud won't be rotated,
	/// so we project them on the plancloud's frame; therefor, it is necessary
	/// that it went through the planarExtractionCell
	/// \pre The cloud must have gone through the planarExtractionCell
	/// \footnote This class still needs to be improved
	planCloudsPtr_t compute(planCloudsPtr_t);
	void rotate(Eigen::Matrix3d& rotation, Eigen::Vector3d& myVector);
	void rotateSurf(Eigen::Matrix3d& rotation, Eigen::Vector3d& center, PlanCloud& planCloud);

private:
	pointCloudPtr_t point_cloud_ptr_;
	int findGroundID(planCloudsPtr_t planCloudListPtr);
};


#endif // ORIENTATIONCELL_H
