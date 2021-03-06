#ifndef PLANCLOUD_H
# define PLANCLOUD_H

# include <fstream>
# include <iostream>
# include <vector>

# include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

# include <eigen3/Eigen/Dense>

# include <pcl/features/normal_3d.h>
# include <pcl/ModelCoefficients.h>
# include <pcl/point_types.h>

#include <vtkSmartPointer.h>
#include <vtkTransform.h>

# include "typedefs.h"
# include "verbose.h"

/// \brief Generic container class for point clouds and their features
class PlanCloud
{
public:
	PlanCloud();
	PlanCloud(const PlanCloud&);
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

	void project_points_on_frame();

	/// Makes sure that the points of the hull convex are sorted in a
	/// trigonometric order
	/// \pre Can only be used for clouds that have been through the
	/// HullConvexCell algorithm
	void set_trigo_order();

	/// Calculates the coordinates of a point compared to the origin and
	/// the frame of the cloud
	/// \pre find_origin() and find_frame() must have been called beforehead
	Eigen::Vector3d project_point(pointCloudPoints_t::size_type i) const;

	/// Prints the objects properties
	std::ostream& print(std::ostream& o) const throw ();
	/// Prints the objects properties with a chosen verbose level
	std::ostream& print(Verbose v, std::ostream& o) const throw ();

	pointCloudPtr_t& cloud ()
	{
		return cloud_;
	}

	const pointCloudPtr_t& cloud () const
	{
		return cloud_;
	}

	pointCloudPtr_t& keyPoints ()
	{
		return keyPoints_;
	}

	const pointCloudPtr_t& keyPoints () const
	{
		return keyPoints_;
	}

	normalCloudPtr_t& normals ()
	{
		return normals_;
	}

	const descriptorCloudPtr_t& descriptors() const
	{
		return descriptors_;
	}

	descriptorCloudPtr_t& descriptors ()
	{
		return descriptors_;
	}

	const normalCloudPtr_t& normals () const
	{
		return normals_;
	}

	pcl::ModelCoefficients::Ptr coefficients()
	{
		return coefficients_;
	}

	const pcl::ModelCoefficients::Ptr coefficients() const
	{
		return coefficients_;
	}

	pointCloudPoints_t::size_type size()
	{
		return cloud_->points.size();
	}

	const boost::optional<Eigen::Vector3d>& origin() const
	{
		return origin_;
	}

	boost::optional<Eigen::Vector3d>& origin()
	{
		return origin_;
	}

	const boost::optional<Eigen::Matrix3d >& frame() const
	{
		return frame_;
	}

	boost::optional<Eigen::Matrix3d >& frame()
	{
		return frame_;
	}

	Eigen::Vector3d& T()
	{
		return T_;
	}

	Eigen::Vector3d& B()
	{
		return B_;
	}

	Eigen::Vector3d& N()
	{
		return N_;
	}

	std::vector<boost::shared_ptr<
		std::pair <std::string, vtkSmartPointer<vtkTransform> > > >& cad_models()
	{
		return cad_models_;
	}

	const std::vector<boost::shared_ptr<
		std::pair <std::string, vtkSmartPointer<vtkTransform> > > >& cad_models() const
	{
		return cad_models_;
	}

private:
	/// The point cloud itself
	pointCloudPtr_t cloud_;

	/// The keypoints of the cloud
	pointCloudPtr_t keyPoints_;

	/// The normals of the point cloud
	normalCloudPtr_t normals_;

	/// The descriptors of the cloud
	descriptorCloudPtr_t descriptors_;

	/// The vector of cad models and their poses in the scene
	std::vector<boost::shared_ptr<std::pair <std::string, vtkSmartPointer<vtkTransform> > > > cad_models_;

	/// Coefficients of the plan carrying the cloud, values are computed by PlanExtractionCell
	pcl::ModelCoefficients::Ptr coefficients_;

	/// Origin of the planCloud (barycenter of the hullConvex)
	boost::optional<Eigen::Vector3d> origin_;

	/// Tangent Vector to the carrying plan
	Eigen::Vector3d T_;

	/// Bi-Tangent Vector to the carrying plan
	Eigen::Vector3d B_;

	/// Normal Vector to the carrying plan
	Eigen::Vector3d N_;

	/// Frame to the carrying plan = [T_, B_, N_]
	boost::optional<Eigen::Matrix3d > frame_;
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
		o << "Surf" << i <<std::endl;
		o << pc->at(i);
	}
	o << std::endl;
	return o;
}

inline std::ostream& operator<< (Verbose v, const PlanCloud& pc)
{
	return pc.print (v, *v._stream);
}

inline std::ostream& operator<<(Verbose v, const planCloudsPtr_t& pc) {
	assert(v._stream);
	std::ostream& o = *v._stream;
	for (pointCloudPoints_t::size_type i = 0;
		 i < pc->size(); i++)
	{
		o << std::endl << "Surf" << i <<std::endl;
		o << v << pc->at(i);
	}
	o << std::endl;
	return o;
}

#endif //! PLANCLOUD_H
