#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "plancloud.h"
#include "typedefs.h"
#include "visualizer.h"

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
	convex_clouds_.push_back(*planCloudList);
}

void Visualizer::add_keypoint_clouds(planCloudsPtr_t planCloudList)
{
	keypoint_groups_.push_back(*planCloudList);
}

void Visualizer::add_normals_clouds(planCloudsPtr_t planCloudList)
{
	normals_groups_.push_back(*planCloudList);
}

void Visualizer::add_cad_model(planCloudsPtr_t planCloudList)
{
	cad_models_.push_back(planCloudList);
}

void Visualizer::display_all()
{

	srand (static_cast<unsigned int> (time(NULL)));
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (255, 255, 255);
	viewer->initCameraParameters ();

//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudColored (new pcl::PointCloud<pcl::PointXYZRGBA>);

//	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> ("../share/cloud-treatment/point-cloud/Scene_Chair_1.pcd", *cloudColored) == -1) //* load the file
//	{
//		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
//	}
//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloudColored);
//	viewer->addPointCloud<pcl::PointXYZRGBA> (cloudColored, rgb, "cloudColored" );
//	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloudColored");



	// displaying normal clouds
	for(unsigned int i = 0; i<cloud_groups_.size(); ++i)
	{
		for(unsigned int j = 0; j<cloud_groups_[i].size(); j++)
		{
			std::string cloudName = "cloud_" + boost::lexical_cast<std::string>(i) + "_"+ boost::lexical_cast<std::string>(j);
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

	// displaying keypoint clouds
	for(unsigned int i = 0; i<keypoint_groups_.size(); ++i)
	{
		for(unsigned int j = 0; j<keypoint_groups_[i].size(); j++)
		{
			std::string cloudName = "keypoints_" + boost::lexical_cast<std::string>(i) + "_"+ boost::lexical_cast<std::string>(j);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(keypoint_groups_[i][j].keyPoints(), 0, 0, 255);
			viewer->addPointCloud<pcl::PointXYZ> (keypoint_groups_[i][j].keyPoints(), single_color,  cloudName);
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, cloudName);
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
//			std::string cloudName = "convex_" + boost::lexical_cast<std::string>(k) + "_" + boost::lexical_cast<std::string>(l);
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

	//displaying normals clouds
	for(unsigned int i = 0; i<normals_groups_.size(); ++i)
	{
		for(unsigned int j = 0; j<normals_groups_[i].size(); j++)
		{
			if(normals_groups_[i][j].normals()->points.size() > 0)
			{
				std::string cloudName = "normals_" +
						boost::lexical_cast<std::string>(i) + "_" +
						boost::lexical_cast<std::string>(j);
				normals_groups_[i][j].cloud()->width =
						static_cast<unsigned int>(
							normals_groups_[i][j].cloud()->points.size());
				normals_groups_[i][j].cloud()->height = 1;
				normals_groups_[i][j].normals()->width =
						static_cast<unsigned int>(
							normals_groups_[i][j].normals()->points.size());
				normals_groups_[i][j].normals()->height = 1;
				viewer->addPointCloudNormals< pcl::PointXYZ, pcl::Normal >
						(normals_groups_[i][j].cloud(),
						 normals_groups_[i][j].normals(), 10, 0.05f, cloudName);
				viewer->setPointCloudRenderingProperties
						(pcl::visualization::PCL_VISUALIZER_COLOR,
						 1.0, 0.0, 0.0,
						 "normals_" + boost::lexical_cast<std::string>(i) +
						 "_" + boost::lexical_cast<std::string>(j));
			}
		}
	}

	for(unsigned int i = 0; i<cad_models_.size(); ++i)
	{
		for(unsigned int j = 0; j<cad_models_[i]->size(); j++)
		{
			for(unsigned int k = 0; k<cad_models_[i]->at(j).cad_models().size(); k++)
			{
				std::string cloudName = "model_" +
						boost::lexical_cast<std::string>(i) + "_" +
						boost::lexical_cast<std::string>(j) + "_" +
						boost::lexical_cast<std::string>(k);

				viewer->addModelFromPLYFile(
							cad_models_[i]->at(j).cad_models().at(k)->first,
							cad_models_[i]->at(j).cad_models().at(k)->second,
							cloudName);
			}
		}
	}
//	viewer->addModelFromPLYFile(
//				"../share/cloud-treatment/cad-model/chair_rotated.ply",
//				"chair_rotated");

	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (10000));
	}
}
