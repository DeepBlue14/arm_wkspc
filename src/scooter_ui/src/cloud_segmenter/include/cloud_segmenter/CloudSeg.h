#ifndef CLOUD_SEG_H
#define CLOUD_SEG_H

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// ROS <--> PCL conversions
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

// PCL
//TODO are all these needed?
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLImage.h>
#include <pcl/io/io.h>
#include <pcl/io/point_cloud_image_extractors.h>
#include <pcl/common/time.h>
#include <pcl/common/angles.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/segmentation/lccp_segmentation.h> //PCL version 1.8.0 or greater

//#define USE_VIEWER

#ifdef USE_VIEWER
#include <pcl/visualization/pcl_visualizer.h>
#endif

//Fun with dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <cloud_segmenter/cloudSegConfig.h>

using namespace ros;
using namespace pcl;

class CloudSeg
{
	private:
		Publisher* cloudPub;
		PointCloud<pcl::PointXYZL>::Ptr lccpSeg(const PointCloud<PointXYZRGB>::ConstPtr& cloud, std::vector<int> indicies);
		pcl::LCCPSegmentation<PointXYZRGB> lccp;
#ifdef USE_VIEWER
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
		//pcl::visualization::PointCloudColorHandlerLabelField < pcl::PointXYZL > label_handler;
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZL> label_handler;
#endif
		void printMembers(std::string preamble);
		float color_importance;
		float spatial_importance;
		float normal_importance;
		bool use_single_cam_transform;
		bool use_supervoxel_refinement;
		bool use_extended_convexity;
		bool use_sanity_criterion;
		bool use_plane_removal;
		uint32_t min_segment_size;
		float concavity_tolerance_threshold;
		float smoothness_threshold;
		float voxel_resolution;
		float seed_resolution;
		void init();
		bool isInit;
	public:
		CloudSeg();
		void cloudCallback(const PointCloud<PointXYZRGB>::ConstPtr& cloud);
		void dynReconfCallback(cloud_segmenter::cloudSegConfig &config, uint32_t level);
		Publisher* getCloudPublisher();
};

#endif /* CLOUD_SEG_H */
