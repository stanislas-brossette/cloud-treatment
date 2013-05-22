#include <boost/make_shared.hpp>

#include "organizedsegmentationcell.h"
#include "plancloud.h"
#include "typedefs.h"

void
displayPlanarRegions (std::vector<pcl::PlanarRegion<pointT>, Eigen::aligned_allocator<pcl::PlanarRegion<pointT> > > &regions,
					  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	char name[1024];
	unsigned char red [6] = {255,   0,   0, 255, 255,   0};
	unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
	unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

	pcl::PointCloud<pointT>::Ptr contour (new pcl::PointCloud<pointT>);

	for (size_t i = 0; i < regions.size (); i++)
	{
		Eigen::Vector3f centroid = regions[i].getCentroid ();
		Eigen::Vector4f model = regions[i].getCoefficients ();
		pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
		pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
										   centroid[1] + (0.5f * model[1]),
										   centroid[2] + (0.5f * model[2]));
		sprintf (name, "normal_%d", unsigned (i));
		viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, name);

		contour->points = regions[i].getContour ();
		sprintf (name, "plane_%02d", int (i));
		pcl::visualization::PointCloudColorHandlerCustom <pointT> color (contour, red[i%6], grn[i%6], blu[i%6]);
		if(!viewer->updatePointCloud(contour, color, name))
			viewer->addPointCloud (contour, color, name);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
	}
}

void
displayEuclideanClusters (const pcl::PointCloud<pointT>::CloudVectorType &clusters,
						  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	char name[1024];
	unsigned char red [6] = {255,   0,   0, 255, 255,   0};
	unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
	unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

	for (size_t i = 0; i < clusters.size (); i++)
	{
		sprintf (name, "cluster_%d" , int (i));
		pcl::visualization::PointCloudColorHandlerCustom<pointT> color0(boost::make_shared<pcl::PointCloud<pointT> >(clusters[i]),red[i%6],grn[i%6],blu[i%6]);
		if (!viewer->updatePointCloud (boost::make_shared<pcl::PointCloud<pointT> >(clusters[i]),color0,name))
			viewer->addPointCloud (boost::make_shared<pcl::PointCloud<pointT> >(clusters[i]),color0,name);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);
	}
}

void
displayCurvature (pcl::PointCloud<pointT>& cloud, pcl::PointCloud<pcl::Normal>& normals, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	pcl::PointCloud<pcl::PointXYZRGBA> curvature_cloud;
	pcl::copyPointCloud(cloud, curvature_cloud);
	for (size_t i  = 0; i < cloud.points.size (); i++)
	{
		if (normals.points[i].curvature < 0.04)
		{
			curvature_cloud.points[i].r = 0;
			curvature_cloud.points[i].g = 255;
			curvature_cloud.points[i].b = 0;
		}
		else
		{
			curvature_cloud.points[i].r = 255;
			curvature_cloud.points[i].g = 0;
			curvature_cloud.points[i].b = 0;
		}
	}

	if (!viewer->updatePointCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >(curvature_cloud), "curvature"))
		viewer->addPointCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >(curvature_cloud), "curvature");

}

void
displayDistanceMap (pcl::PointCloud<pointT>& cloud, float* distance_map, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
	pcl::PointCloud<pcl::PointXYZRGBA> distance_map_cloud;
	pcl::copyPointCloud(cloud, distance_map_cloud);
	for (size_t i  = 0; i < cloud.points.size (); i++)
	{
		if (distance_map[i] < 5.0)
		{
			distance_map_cloud.points[i].r = 255;
			distance_map_cloud.points[i].g = 0;
			distance_map_cloud.points[i].b = 0;
		}
		else
		{
			distance_map_cloud.points[i].r = 0;
			distance_map_cloud.points[i].g = 255;
			distance_map_cloud.points[i].b = 0;
		}
	}

	if (!viewer->updatePointCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >(distance_map_cloud), "distance_map"))
		viewer->addPointCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA> >(distance_map_cloud), "distance_map");
}

OrganizedSegmentationCell::OrganizedSegmentationCell():
	Cell()
{
	parameters()["name"] = "OrganizedSegmentationCell";

	parameters()["display_normals"] = 0;
	parameters()["display_curvature"] = 0;
	parameters()["display_distance_map"] = 0;

	parameters()["use_planar_refinement"] = 1;
	parameters()["use_clustering"] = 1;

	parameters()["plane_min_inliers"] = 10000;
	parameters()["plane_angular_threshold"] = 3.0;
	parameters()["plane_distance_threshold"] = 0.02;

	parameters()["euclidian_cluster_distance_threshold"] = 0.01;
	parameters()["euclidian_cluster_min_size"] = 1000;

	parameters()["edge_aware_distance_threshold"] = 0.01;

	ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
	ne.setMaxDepthChangeFactor (0.02f);
	ne.setNormalSmoothingSize (20.0f);

	plane_comparator_.reset (new pcl::PlaneCoefficientComparator<pointT, pcl::Normal> ());
	euclidean_comparator_.reset (new pcl::EuclideanPlaneCoefficientComparator<pointT, pcl::Normal> ());
	//	rgb_comparator_.reset (new pcl::RGBPlaneCoefficientComparator<pointT, pcl::Normal> ());
	edge_aware_comparator_.reset (new pcl::EdgeAwarePlaneComparator<pointT, pcl::Normal> ());
	euclidean_cluster_comparator_ = pcl::EuclideanClusterComparator<pointT, pcl::Normal, pcl::Label>::Ptr (new pcl::EuclideanClusterComparator<pointT, pcl::Normal, pcl::Label> ());
}

planCloudsPtr_t OrganizedSegmentationCell::compute(
		planCloudsPtr_t planCloudListPtr)
{
	cell_name_ = boost::get<std::string>(parameters()["name"]);

	display_normals_ = static_cast<int>(boost::get<double>(parameters()["display_normals"]));
	display_curvature_ = static_cast<int>(boost::get<double>(parameters()["display_curvature"]));
	display_distance_map_ = static_cast<int>(boost::get<double>(parameters()["display_distance_map"]));
	use_planar_refinement_ = static_cast<int>(boost::get<double>(parameters()["use_planar_refinement"]));
	use_clustering_ = static_cast<int>(boost::get<double>(parameters()["use_clustering"]));
	plane_min_inliers_ = static_cast<uint>(boost::get<double>(parameters()["plane_min_inliers"]));
	plane_angular_threshold_ = static_cast<float>(boost::get<double>(parameters()["plane_angular_threshold"]));
	plane_distance_threshold_ = static_cast<float>(boost::get<double>(parameters()["plane_distance_threshold"]));

	euclidian_cluster_distance_threshold_ = static_cast<float>(boost::get<double>(parameters()["euclidian_cluster_distance_threshold"]));
	euclidian_cluster_min_size_ = static_cast<uint>(boost::get<double>(parameters()["euclidian_cluster_min_size"]));
	edge_aware_distance_threshold_ = static_cast<float>(boost::get<double>(parameters()["edge_aware_distance_threshold"]));


	// Set up Organized Multi Plane Segmentation
	mps.setMinInliers (plane_min_inliers_);
	mps.setAngularThreshold (pcl::deg2rad (plane_angular_threshold_)); //3 degrees
	mps.setDistanceThreshold (plane_distance_threshold_); //2cm

	for(size_t i = 0; i < planCloudListPtr->size(); ++i)
	{
		const CloudConstPtr& cloud = planCloudListPtr->at(i).cloud();
		// Estimate Normals
		pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (
					new pcl::PointCloud<pcl::Normal>);
		ne.setInputCloud (cloud);
		ne.compute (*normal_cloud);
		float* distance_map = ne.getDistanceMap ();
		boost::shared_ptr<pcl::EdgeAwarePlaneComparator<pointT,pcl::Normal> >
				eapc = boost::dynamic_pointer_cast<
				pcl::EdgeAwarePlaneComparator<pointT,pcl::Normal> >(
					edge_aware_comparator_);
		eapc->setDistanceMap (distance_map);
		eapc->setDistanceThreshold (edge_aware_distance_threshold_, false);

		// Segment Planes
		double mps_start = pcl::getTime ();
		std::vector<pcl::PlanarRegion<pointT>,
				Eigen::aligned_allocator<pcl::PlanarRegion<pointT> > > regions;
		std::vector<pcl::ModelCoefficients> model_coefficients;
		std::vector<pcl::PointIndices> inlier_indices;
		pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
		std::vector<pcl::PointIndices> label_indices;
		std::vector<pcl::PointIndices> boundary_indices;
		mps.setInputNormals (normal_cloud);
		mps.setInputCloud (cloud);

		if (use_planar_refinement_)
		{
			std::cout << "Using planar refinement\n";
			mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
		}
		else
		{
			std::cout << "Not using planar refinement\n";
			mps.segment (regions);//, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
		}
		double mps_end = pcl::getTime ();
		std::cout << "MPS+Refine took: " << double(mps_end - mps_start) << std::endl;

		//Segment Objects
		pcl::PointCloud<pointT>::CloudVectorType clusters;

		if (use_clustering_ && regions.size () > 0)
		{
			std::vector<bool> plane_labels;
			plane_labels.resize (label_indices.size (), false);
			for (size_t i = 0; i < label_indices.size (); i++)
			{
				if (label_indices[i].indices.size () > 10000)
				{
					plane_labels[i] = true;
				}
			}

			euclidean_cluster_comparator_->setInputCloud (cloud);
			euclidean_cluster_comparator_->setLabels (labels);
			euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
			euclidean_cluster_comparator_->setDistanceThreshold (
						euclidian_cluster_distance_threshold_, false);

			pcl::PointCloud<pcl::Label> euclidean_labels;
			std::vector<pcl::PointIndices> euclidean_label_indices;
			pcl::OrganizedConnectedComponentSegmentation<pointT,pcl::Label>
					euclidean_segmentation (euclidean_cluster_comparator_);
			euclidean_segmentation.setInputCloud (cloud);
			euclidean_segmentation.segment (
						euclidean_labels, euclidean_label_indices);

			for (size_t i = 0; i < euclidean_label_indices.size (); i++)
			{
				if (euclidean_label_indices[i].indices.size () > euclidian_cluster_min_size_)
				{
					std::cout << euclidean_label_indices[i].indices.size () << " > " << euclidian_cluster_min_size_ << std::endl;
					pcl::PointCloud<pointT> cluster;
					pcl::copyPointCloud (
								*cloud,euclidean_label_indices[i].indices,cluster);
					clusters.push_back (cluster);
				}
			}

			PCL_INFO ("Got %d euclidean clusters!\n", clusters.size ());
		}


		vis_ = boost::make_shared<pcl::visualization::PCLVisualizer> ("3D Viewer");
		vis_->initCameraParameters ();

		vis_->addPointCloud (cloud, "cloud");

		vis_->resetCameraViewpoint ("cloud");

		displayPlanarRegions (regions, vis_);
		displayEuclideanClusters (clusters,vis_);
		pcl::PointCloud<pointT> prev_cloud_ = *cloud;
		pcl::PointCloud<pcl::Normal> prev_normals_ = *normal_cloud;
		float* prev_distance_map_ = distance_map;
		if (display_curvature_)
			displayCurvature (prev_cloud_, prev_normals_, vis_);
		if (display_distance_map_)
			displayDistanceMap (prev_cloud_, prev_distance_map_, vis_);

		while (!vis_->wasStopped ())
		{
			vis_->spinOnce (100);
			boost::this_thread::sleep (boost::posix_time::microseconds (1000));
		}

	}


	return planCloudListPtr;
}
