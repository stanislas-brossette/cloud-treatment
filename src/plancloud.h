#ifndef PLANCLOUD_H
# define PLANCLOUD_H

# include <iostream>
# include <fstream>
# include <vector>
#include <eigen3/Eigen/Dense>
# include <pcl/point_types.h>
# include <pcl/ModelCoefficients.h>
#include <boost/optional.hpp>

# include "typedefs.h"

/// \brief Generic container class for point clouds and their features
class PlanCloud
{
public:
	PlanCloud();
	virtual ~PlanCloud ()
	{
	}

	/// Resets the pointCloud attribute.
	void reset();

	/// Finds the origin of the cloud by calculating the average point
	void find_origin();

	/// Calculates the frame of the cloud:
	/// Tangent, Bitangent and Normal vectors.
	/// \pre Can only be used for clouds that have been through the
	/// PlanExtractionCell algorithm
	void find_frame();

	/// Calculates the coordinates of a point compared to the origin and
	/// the frame of the cloud
	/// \pre find_origin() and find_frame() must have been called beforehead
	Eigen::Vector3d project_point(pointCloudPoints_t::size_type i);

	/// Prints the objects properties
	std::ostream& print(std::ostream& o) const throw ();

	pointCloudPtr_t& cloud ()
	{
		return cloud_;
	}

	const pointCloudPtr_t& cloud () const
	{
		return cloud_;
	}

	pcl::ModelCoefficients::Ptr coefficients()
	{
		return coefficients_;
	}

	const pcl::ModelCoefficients::Ptr coefficients() const
	{
		return coefficients_;
	}

	pointCloud2Ptr_t cloud2()
	{
		return cloud2_;
	}

	const pointCloud2Ptr_t cloud2() const
	{
		return cloud2_;
	}

	pointCloudPoints_t::size_type size()
	{
		return cloud_->points.size();
	}

	const boost::optional<Eigen::Vector3d> origin() const
	{
		return origin_;
	}

	const boost::optional<std::vector<Eigen::Vector3d> > frame() const
	{
		return frame_;
	}

private:
	pointCloudPtr_t cloud_;
	pointCloud2Ptr_t cloud2_;
	pcl::ModelCoefficients::Ptr coefficients_;
	boost::optional<Eigen::Vector3d> origin_;
	boost::optional<std::vector<Eigen::Vector3d> > frame_;
};

inline std::ostream& operator<< (std::ostream& o, const PlanCloud& pc)
{
	return pc.print (o);
}

inline std::ostream& operator<< (std::ostream& o, const planCloudsPtr_t& pc)
{
	for (pointCloudPoints_t::size_type i = 0;
		 i < pc->size(); i++)
	{
		o << pc->at(i);
	}
	o << std::endl;
	return o;
}

#endif //! PLANCLOUD_H
