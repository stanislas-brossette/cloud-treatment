#ifndef PLANCLOUD_H
# define PLANCLOUD_H

# include <iostream>
# include <fstream>
# include <pcl/point_types.h>
# include <pcl/ModelCoefficients.h>
# include <sensor_msgs/PointCloud2.h>

# include "typedefs.h"

///Generic container class for point clouds + their carrying plan
class PlanCloud
{
public:
	PlanCloud();
	void display_cloud();
	void info();
	void display_planar_components();
	void reset();
	void update_cloud2_from_cloud();
	void update_cloud_from_cloud2();

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

private:
	pointCloudPtr_t cloud_;
	pointCloud2Ptr_t cloud2_;
	pcl::ModelCoefficients::Ptr coefficients_;
};

#endif //! PLANCLOUD_H
