#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <boost/shared_ptr.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include "plancloud.h"
#include "typedefs.h"

/// \brief Visualization class
///
/// This class allows to display several clouds in the same instance

struct color
{
	color(unsigned int R, unsigned int G, unsigned int B)
	{
		r = R;
		g = G;
		b = B;
	}

	color(){}

	unsigned int r;
	unsigned int g;
	unsigned int b;
};

class Visualizer
{
public:
	Visualizer();
	virtual ~Visualizer ()
	{
	}

	void add_xyz_clouds(planCloudsPtr_t planCloudList);
	void add_xyzrgba_clouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr);
	void add_convex_clouds(planCloudsPtr_t planCloudList);
	void add_keypoint_clouds(planCloudsPtr_t planCloudList);
	void add_normals_clouds(planCloudsPtr_t planCloudList);
	void display_all();

	std::vector<color> colors;


private:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
	std::vector<planClouds_t> cloud_groups_;
	std::vector<planClouds_t> keypoint_groups_;
	std::vector<planClouds_t> normals_groups_;
	std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > > color_clouds_;
	std::vector<planClouds_t> convex_clouds_;
};

#endif // VISUALIZER_H
