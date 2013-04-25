#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

#include "visualizer.h"
#include "plancloud.h"
#include "typedefs.h"

Visualizer::Visualizer()
{
	colors.push_back(color(17, 63, 140));
	colors.push_back(color(1, 164, 164));
	colors.push_back(color(0, 161, 203));
	colors.push_back(color(97, 174, 36));
	colors.push_back(color(208, 209, 2));
	colors.push_back(color(50, 116, 44));
	colors.push_back(color(215, 0, 96));
	colors.push_back(color(229, 64, 40));
	colors.push_back(color(241, 141, 5));
	colors.push_back(color(34,64,42));
	colors.push_back(color(126,165,51));
	colors.push_back(color(236,236,50));
	colors.push_back(color(132,161,120));
}

void Visualizer::add_xyz_clouds(planCloudsPtr_t planCloudList)
{
	planClouds_t newPlanCloudList;
	for (unsigned int i = 0; i<planCloudList->size(); ++i)
	{
		newPlanCloudList.push_back(planCloudList->at(i));
	}
	cloud_groups_.push_back(*planCloudList);
}

void Visualizer::add_xyzrgba_clouds(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RGBCloud)
{
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA> > copyRGBCloud;
	copyRGBCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> > ();
	*copyRGBCloud = *RGBCloud;
	color_clouds_.push_back(copyRGBCloud);
}

void Visualizer::add_convex_clouds(planCloudsPtr_t planCloudList)
{
	planClouds_t newPlanCloudList;
	for (unsigned int i = 0; i<planCloudList->size(); ++i)
	{
		newPlanCloudList.push_back(planCloudList->at(i));
	}
	convex_clouds_.push_back(*planCloudList);
}

void Visualizer::display_all()
{
	srand (static_cast<unsigned int> (time(NULL)));
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (255, 255, 255);
	viewer->initCameraParameters ();
	// displaying normal clouds
	for(unsigned int i = 0; i<cloud_groups_.size(); ++i)
	{
		for(unsigned int j = 0; j<cloud_groups_[i].size(); j++)
		{
			std::string cloudName = "cloud" + boost::lexical_cast<std::string>(i) + "_"+ boost::lexical_cast<std::string>(j);
//			int r = rand() % 100 + 100;
//			int g = rand() % 100 + 100;
//			int b = rand() % 100 + 100;
//			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_groups_[i][j].cloud(), r, g, b);
			color Color;
			if(cloud_groups_[i].size() == 1)
			{
				Color.r = 140;
				Color.g = 138;
				Color.b = 134;
			}
			else
			{
				unsigned int colorIndex = j + 2;//static_cast<int>(rand() % colors.size());
				Color.r = colors[colorIndex].r;
				Color.g = colors[colorIndex].g;
				Color.b = colors[colorIndex].b;
			}
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_groups_[i][j].cloud(), Color.r, Color.g, Color.b);
			viewer->addPointCloud<pcl::PointXYZ> (cloud_groups_[i][j].cloud(), single_color,  cloudName);
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, cloudName);
		}
	}

	// displaying color clouds
	for(unsigned int i = 0; i<color_clouds_.size(); ++i)
	{
		std::string cloudName = "cloudColor" + boost::lexical_cast<std::string>(i);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(color_clouds_[i]);
		viewer->addPointCloud<pcl::PointXYZRGBA> (color_clouds_[i], rgb, cloudName);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudName);
	}

	// displaying convex clouds
	for(unsigned int k = 0; k<convex_clouds_.size(); ++k)
	{
		for(unsigned int l = 0; l<convex_clouds_[k].size(); l++)
		{
			std::string name = "convex" + boost::lexical_cast<std::string>(l);
			pcl::PointXYZ textPoint;
			if(!convex_clouds_[k][l].origin())
			{
				convex_clouds_[k][l].find_origin();
			}
			textPoint.x = static_cast<float>(convex_clouds_[k][l].origin()->x());
			textPoint.y = static_cast<float>(convex_clouds_[k][l].origin()->y());
			textPoint.z = static_cast<float>(convex_clouds_[k][l].origin()->z());
//			viewer->addText3D(name, textPoint, 0.03, 0, 0, 0, name);
			pointCloudPoints_t::size_type j = convex_clouds_[k][l].cloud()->points.size() - 1;
			std::string id = "";
			for(pointCloudPoints_t::size_type i = 0; i < convex_clouds_[k][l].cloud()->points.size(); ++i)
			{
				id = "Line" + boost::lexical_cast<std::string>(i)+ boost::lexical_cast<std::string>(j)+ boost::lexical_cast<std::string>(k)+ boost::lexical_cast<std::string>(l);
				viewer->addLine<pcl::PointXYZ> (convex_clouds_[k][l].cloud()->points[j], convex_clouds_[k][l].cloud()->points[i], 255, 0, 0, id);
				j = i;
			}
		}
	}

	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (10000));
	}
}
