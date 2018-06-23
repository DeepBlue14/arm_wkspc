#include <ros/ros.h>
#include "transform_cloud/transform_cloud.h"

CloudTransformer::CloudTransformer(std::string fromTopic, std::string toTopic, std::string toFrame, ros::NodeHandle& nh) :
		to(toTopic), from(fromTopic), frame(toFrame), nh(nh)
{
}

void CloudTransformer::init()
{
	pub = nh.advertise < pcl::PointCloud<pcl::PointXYZRGB> > (to, 1);
	sub = nh.subscribe < pcl::PointCloud < pcl::PointXYZRGB > ::Ptr > (from, 1, &CloudTransformer::cloudCallback, this);
	//Note that this looks the transform exactly once, and we don't notice if it changes later.
	//This is fine for things that don't move relative to each other, but wrong for moving links
	while (!transform_is_init)
	{
		//Wait up to two seconds for transform, if you don't get it, continue
		if (tf_l.waitForTransform(cloud->header.frame_id, frame, ros::Time(0), ros::Duration(2.0)))
		{
			tf_l.lookupTransform(cloud->header.frame_id, frame, ros::Time(0), transform);
			transform_is_init = true;
		}
		else
		{
			ROS_WARN("No transform from %s to %s arrived in 2 seconds, retrying", cloud->header.frame_id.c_str(), frame.c_str());
			return;
		}
	}
}

void CloudTransformer::cloudCallback(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	//just save the cloud
	this->cloud = cloud;
}

void CloudTransformer::update()
{
	//cloud is a shared pointer, and so is "false" until initialized, which happens in cloudCallback
	if(transform_is_init && cloud)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		temp_cloud->header.frame_id = frame;

		//Args are frame to transform to, input, output, and transform listener to use
		//So after this, temp_cloud is the cloud the iterator points to, transformed into the frame of the first cloud
		pcl_ros::transformPointCloud(frame, *cloud, *(temp_cloud), tf_l);
		pub.publish(temp_cloud);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "select_grasps");
	ros::NodeHandle node("~");

	// Read the parameters from the launchfile
	std::string from, to, to_frame;
	// Topic to listen to
	node.param < std::string > ("/transform_cloud/from_topic", from, "/rgbd_cam_1/depth_registered/points");
	// Topic to publish on
	node.param < std::string > ("/transform_cloud/to_topic", to, "/transform_cloud/transformed");
	// Frame to transform to
	node.param < std::string > ("/transform_cloud/to_frame", to_frame, "/base");
	// Create transformer object
	CloudTransformer ct(from, to, to_frame, node);
	ct.init();

	//Don't transform every cloud (causes v. high CPU use), just update once a second
	ros::Rate rate(0.5);
	while (ros::ok())
	{
		ros::spinOnce();
		ct.update();
		rate.sleep();
	}

}
