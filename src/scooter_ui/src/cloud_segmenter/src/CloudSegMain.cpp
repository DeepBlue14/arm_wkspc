#include "cloud_segmenter/CloudSeg.h"

using namespace ros;
using namespace std;

int main(int argc, char **argv)
{
	string nodeName = "cloud_segmenter";
	init(argc, argv, nodeName);

	ROS_INFO("Starting node\n");

	CloudSeg cloudSegmenter;
	NodeHandle nh("~");

	//Get the topic for the cloud
	string cloudTopic;
	if (nh.getParam("cloud_topic", cloudTopic))
	{
		ROS_INFO("Node %s listening to %s", nodeName.c_str(), cloudTopic.c_str());
	}
	else
	{
		nh.setParam("cloud_topic", "/unified_cloud");
		ROS_WARN("Cloud topic not set, defaulted to %s", cloudTopic.c_str());
	}

	//Subscribe to the specified cloud topic
	Subscriber pointSub = nh.subscribe<const PointCloud<PointXYZRGB>::ConstPtr&>(cloudTopic, 1, &CloudSeg::cloudCallback, &cloudSegmenter);

	//Set the publisher name based on this node's name, start publishing labled clouds
	Publisher* cloudPub = cloudSegmenter.getCloudPublisher();
	*cloudPub = nh.advertise < pcl::PointCloud<pcl::PointXYZL> > ("/" + nodeName + "/depth_labled/points", 1);

	//Set up the dynamic reconfigurer
	dynamic_reconfigure::Server<cloud_segmenter::cloudSegConfig> server;
	dynamic_reconfigure::Server<cloud_segmenter::cloudSegConfig>::CallbackType cbk;
	cbk = boost::bind(&CloudSeg::dynReconfCallback, &cloudSegmenter, _1, _2);
	server.setCallback(cbk);

	spin();

	return EXIT_SUCCESS;
}
