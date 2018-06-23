/*
 * Definitions for the object filter. The object filter listens for a segmented cloud (with labels)
 * and a laser point, and cuts the segmented cloud down to only the segment with the laser point in it.
 */

#ifndef OBJECT_FILTER_H
#define OBJECT_FILTER_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>
#include <limits>

class ObjectFilter {
protected:
	tf::TransformListener tf_l;
	tf::StampedTransform transform;
	bool transform_is_init;
	std::string frame;
	geometry_msgs::PointConstPtr pixelPoint;
	geometry_msgs::Point realWorldCoorPoint;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	ros::Publisher point3D_pub;
	bool havePointCloud;
	bool haveLaserPoint;
public:
	bool haveAllData();
	void clearState();
	ObjectFilter();
	void pointCallback(const geometry_msgs::PointConstPtr& pixelPoint);
	void pointCloudCallback(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	ros::Publisher* getPointPublisher();
	void filterCloud();
	~ObjectFilter() {};	//Doesn't do anything (except prevent a linker error)
};

class PubSubManager {
protected:
	ObjectFilter* objectFilter;
	ros::NodeHandle nodeHandle;
	ros::Subscriber pointSub;
	ros::Subscriber pointCloudSub;
	ros::Publisher* pointPub;
	std::string laser_topic, cloud_topic;
public:
	void subscribeAll(ObjectFilter* objf, ros::NodeHandle nh, std::string myNodeName);
	void cmdCallback(std_msgs::Int16 command);
};
#endif //OBJECT_FILTER_H
