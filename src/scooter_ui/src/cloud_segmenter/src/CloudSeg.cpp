#include "cloud_segmenter/CloudSeg.h"

#ifdef USE_VIEWER
CloudSeg::CloudSeg() :
viewer(new visualization::PCLVisualizer("Segmentation Viewer"))
#else
CloudSeg::CloudSeg()
#endif
{
	cloudPub = new Publisher();
	pcl::PointCloud<PointXYZRGB>::Ptr init_cloud_ptr(new pcl::PointCloud<
			PointXYZRGB>);

	isInit = false;
	//ROS_ERROR("CONS CALLED");
}

Publisher* CloudSeg::getCloudPublisher()
{
	return cloudPub;
}

void CloudSeg::printMembers(std::string preamble)
{
	ROS_WARN("%s", preamble.c_str());
	ROS_WARN("%p", (void *) this);
	if (isInit)
	{
		ROS_WARN("Is initialized");
	}

	if (use_single_cam_transform)
	{
		ROS_WARN("Using single camera transform");
	}
	if (use_supervoxel_refinement)
	{
		ROS_WARN("Using supervoxel refinement");
	}
	if (use_extended_convexity)
	{
		ROS_WARN("Using extended convexity");
	}
	if (use_sanity_criterion)
	{
		ROS_WARN("Using sanity criterion");
	}
	ROS_WARN("color importance: %f", color_importance);
	ROS_WARN("spatial importance: %f", spatial_importance);
	ROS_WARN("normal importance %f", normal_importance);
	ROS_WARN("concavity tolerance %f", concavity_tolerance_threshold);
	ROS_WARN("smoothness threshold %f", smoothness_threshold);
	ROS_WARN("voxel resolution %f", voxel_resolution);
	ROS_WARN("seed resolution %f", seed_resolution);
	ROS_WARN("min segment size %i", min_segment_size);
	ROS_WARN("------");
}

void CloudSeg::dynReconfCallback(cloud_segmenter::cloudSegConfig &config, uint32_t level)
{
	//Apparently works by blowing everything away and restarting?
	//init();
	//No, it apparently has two objects. One gets this callback, the other gets cloud callbacks

	//TODO THIS IS SUPERUNTHREADSAFE
	//configure the supervoxelizer
	color_importance = config.color_importance;
	spatial_importance = config.spatial_importance;
	normal_importance = config.normal_importance;
	use_supervoxel_refinement = config.use_supervox_refinement;

	// Supervoxel Stuff
	voxel_resolution = config.voxel_resolution;
	seed_resolution = config.seed_resolution;

	//Configure the LCCP segmenter
	lccp.setConcavityToleranceThreshold(config.concavity_thresh);
	lccp.setSanityCheck(config.use_sanity_criterion);
	use_extended_convexity = config.use_extended_convexity;
	uint k_factor = 0;
	if (use_extended_convexity)
	{
		k_factor = 1;
	}
	lccp.setKFactor(k_factor);
	//Always does smoothness check (at the moment, anyway)
	lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, config.smoothness_thresh);
	lccp.setKFactor(k_factor);

	use_plane_removal = config.use_plane_removal;

	//printMembers("-> After reconfigure:");
}

void CloudSeg::init()
{
	//The importance values are floating point unitless weights. 0 is "don't use this parameter", otherwise they are relative to each other
	//The equation they go into is euclidian distance in a 3D space with color, distance, and angle as dimensions
	color_importance = 0.0f; //Weight for color distance
	spatial_importance = 1.0f; //Weight for spatial distance
	normal_importance = 4.0f; //Weight for angle between surface normals
	use_single_cam_transform = false;
	use_supervoxel_refinement = false;

	// Supervoxel Stuff
	voxel_resolution = 0.0075f;
	seed_resolution = 0.03f;

	// LCCPSegmentation Stuff
	concavity_tolerance_threshold = 10; //To ignore small concavities, larger ignores large concavities (probably in degrees)
	smoothness_threshold = 0.1; //probably smoothing to deal with "holes" in transparent or reflective objects
	min_segment_size = 0;
	use_extended_convexity = false; // concave points can be convex if they have convex connections to adjacent points
	use_sanity_criterion = false; //true to break objects at singular connections
	use_plane_removal = false; //Don't usually remove planes

	//Set up k factor for LCCP
	uint k_factor = 0; //if you want to use extended convexity, set to 1
	if (use_extended_convexity)
	{
		k_factor = 1;
	}

	//Set up LCCP object
	lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
	lccp.setSanityCheck(use_sanity_criterion);
	lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
	lccp.setKFactor(k_factor);

#ifdef USE_VIEWER
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
#endif

	//Done initializing
	isInit = true;
	//printMembers("-> After init:");
}

pcl::PointCloud<pcl::PointXYZL>::Ptr CloudSeg::lccpSeg(const PointCloud<
		PointXYZRGB>::ConstPtr& cloud, std::vector<int> plane_indices)
{

	//printMembers("-> Before LCCP:");

	pcl::SupervoxelClustering < PointXYZRGB > superVox(voxel_resolution, seed_resolution);
	//Set up superVoxel object
	superVox.setUseSingleCameraTransform(use_single_cam_transform);
	superVox.setColorImportance(color_importance);
	superVox.setSpatialImportance(spatial_importance);
	superVox.setNormalImportance(normal_importance);

	superVox.setInputCloud(cloud);

	//Get supervoxels
	std::map<uint32_t, pcl::Supervoxel<PointXYZRGB>::Ptr> supervoxel_clusters;
	superVox.extract(supervoxel_clusters);

	if (use_supervoxel_refinement)
	{
		superVox.refineSupervoxels(2, supervoxel_clusters);
	}

	std::multimap < uint32_t, uint32_t > supervoxel_adjacency;
	superVox.getSupervoxelAdjacency(supervoxel_adjacency);

	// Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visulization)
	pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering < PointXYZRGB > ::makeSupervoxelNormalCloud(supervoxel_clusters);

	// The Main Step: Perform LCCPSegmentation
	lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
	lccp.segment();

	if (min_segment_size > 0)
	{
		lccp.setMinSegmentSize(min_segment_size);
		//Explicit call not needed, done automatically (I think)
		//lccp.mergeSmallSegments();
	}

	// Label the cloud and return it
	pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = superVox.getLabeledCloud();
	pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
	lccp.relabelCloud(*lccp_labeled_cloud);

	// Label all the points in the plane as part of the "points not in any segment" cluster
	for (std::vector<int>::iterator planeIt = plane_indices.begin();
			planeIt != plane_indices.end(); planeIt++)
	{
		lccp_labeled_cloud->points[(*planeIt)].label = 0;
	}

#ifdef USE_VIEWER
	if (!viewer->wasStopped())
	{
		viewer->spinOnce();

		//Cut down the point cloud to points within 1.1 meters
		pcl::PassThrough < pcl::PointXYZL > pass_x;
		pass_x.setInputCloud(lccp_labeled_cloud);
		pass_x.setFilterFieldName("x");
		pass_x.setFilterLimits(0, 1.1);
		//pass_x.setFilterLimitsNegative (true);
		pcl::PointCloud<pcl::PointXYZL>::Ptr x_filtered(new pcl::PointCloud<pcl::PointXYZL>);
		pass_x.filter(*x_filtered);

		//if (!viewer->updatePointCloud(lccp_labeled_cloud, label_handler, "labeled cloud"))
		if (!viewer->updatePointCloud(x_filtered, "labeled cloud"))
		{
			//viewer->addPointCloud < pcl::PointXYZL > (lccp_labeled_cloud, label_handler, "labeled cloud");
			viewer->addPointCloud < pcl::PointXYZL > (x_filtered, "labeled cloud");
			//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "labeled cloud");
			//viewer->resetCameraViewpoint("labeled cloud");
			ROS_WARN("Updated cloud viewer");
		}
	}
#endif
	return lccp_labeled_cloud;

}

void CloudSeg::cloudCallback(const PointCloud<PointXYZRGB>::ConstPtr& input)
{
	//Don't segment unless init is done
	if (!isInit)
	{
		init();
	}

	if (use_plane_removal)
	{
		// Create the segmentation object for the planar model and set all the parameters
		SACSegmentation < PointXYZRGB > seg;
		PointIndices::Ptr inliers(new PointIndices);
		ModelCoefficients::Ptr coefficients(new ModelCoefficients);
		seg.setOptimizeCoefficients(true);
		seg.setModelType(SACMODEL_PLANE);
		seg.setMethodType(SAC_RANSAC);
		seg.setMaxIterations(100);
		seg.setDistanceThreshold(0.01);

		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(input);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			ROS_WARN("Using plane extraction, but could not estimate a planar model for the given dataset.");
		}

		// Extract the planar inliers from the input cloud
		//ExtractIndices < PointXYZRGB > extract;
		//extract.setInputCloud(input);
		//extract.setIndices(inliers);

		// Remove the planar inliers, extract the rest
		//extract.setNegative(true);
		//PointCloud<PointXYZRGB>::Ptr cloud_f (new PointCloud<PointXYZRGB>);
		//extract.filter(*cloud_f);
		//Segment the incoming cloud and publish the result
		//TODO we may want to throttle this
		if (!(input->size() == 0))
		{
			PointCloud<PointXYZL>::Ptr resultCloud = lccpSeg(input, inliers->indices);
			cloudPub->publish(resultCloud);
		}
	}
	else
	{
		//Segment the incoming cloud and publish the result
		//TODO we may want to throttle this
		if (!(input->size() == 0))
		{
			PointCloud<PointXYZL>::Ptr resultCloud = lccpSeg(input, std::vector<int>(0));
			cloudPub->publish(resultCloud);
		}
	}
}

