#ifndef TYPEDEFS_H
# define TYPEDEFS_H

# include <pcl/point_cloud.h>
# include <pcl/point_types.h>

class PlanCloud;

/// List of usefull typedef

typedef pcl::PointCloud<pcl::PointXYZ> pointCloud_t;
typedef pointCloud_t::VectorType pointCloudPoints_t;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr_t;
typedef pointCloudPoints_t::size_type pointCloudSize_t;

typedef std::vector<PlanCloud> planClouds_t;
typedef boost::shared_ptr<PlanCloud > planCloudPtr_t;
typedef boost::shared_ptr<planClouds_t> planCloudsPtr_t;
typedef boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pointCloudPtr_t;

typedef pcl::PointCloud <pcl::Normal> normalCloud_t;
typedef boost::shared_ptr<pcl::PointCloud <pcl::Normal> > normalCloudPtr_t;

# endif
