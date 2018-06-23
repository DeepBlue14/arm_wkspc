#include "object_filter/ObjFilter.h"

void PubSubManager::subscribeAll(ObjectFilter* objf, ros::NodeHandle nh, std::string myNodeName)
{
	//Read the parameters: topics to read and the size of the area around the laser pointer
	// Topic to listen to
	double threshold_radius = 0.0;
	if (nh.getParam("laser_topic", laser_topic))
	{
		ROS_INFO("Cloud filter listening to %s", laser_topic.c_str());
	}
	else
	{
		nh.setParam("laser_topic", "/scooter/geometry_msgs/center_point");
		ROS_WARN("Cloud topic not set, defaulted to %s", laser_topic.c_str());
	}

	if (nh.getParam("cloud_topic", cloud_topic))
	{
		ROS_INFO("Cloud filter listening to %s", cloud_topic.c_str());
	}
	else
	{
		nh.setParam("cloud_topic", "/unified_cloud");
		ROS_WARN("Cloud topic not set, defaulted to %s", cloud_topic.c_str());
	}

	//Subscribe it to the relevant topics
	pointSub = nh.subscribe < geometry_msgs::Point > (laser_topic, 1, &ObjectFilter::pointCallback, objf);
	pointCloudSub = nh.subscribe < pcl::PointCloud < pcl::PointXYZRGB > ::Ptr > (cloud_topic, 1, &ObjectFilter::pointCloudCallback, objf);

	//Set up the ObjectFilter point publisher
	pointPub = objf->getPointPublisher();
	*pointPub = nh.advertise <geometry_msgs::Point> ("/"+ myNodeName +"/point", 1);

	//Keep the ObjectFilter and node handle for later use in resubscriptions
	objectFilter = objf;
	nodeHandle = nh;
}

void PubSubManager::cmdCallback(std_msgs::Int16 command)
{
	if(command.data == 1)
	{
		//Subscribe to the topic
		pointCloudSub = nodeHandle.subscribe < pcl::PointCloud < pcl::PointXYZRGB > ::Ptr > (cloud_topic, 1, &ObjectFilter::pointCloudCallback, objectFilter);

		//invalidate old data
		objectFilter->clearState();

		ROS_WARN("Waiting for all data to arrive");
		//Wait for new data to arrive
		ros::Rate dataWait(50);
		while(!objectFilter->haveAllData())
		{
			//Do nothing, waiting for point cloud to arrive
			ros::spinOnce();
			dataWait.sleep();
		}

		//unsubscribe
		pointCloudSub.shutdown();
		ROS_WARN("Got all the data, finding point now");
		//Publish the message containing the 3D point corresponding to the laser
		objectFilter->filterCloud();

	}
	else
	{
		ROS_WARN("Got invalid command %d", command.data);
	}
}

int main(int argc, char **argv)
{
	//Start up my node
	std::string myNodeName = "object_filter";
	ros::init(argc, argv, myNodeName);
	ROS_INFO("Starting laser point object filter");
	ros::NodeHandle nh("~");

	//Set up the grasp filter
	ObjectFilter objf;

	//Set up subscriptions for the ObjectFilter
	PubSubManager psm;
	psm.subscribeAll(&objf, nh, myNodeName);

	//Subscriber for commands to unsubscribe the point cloud subscriber
	ros::Subscriber commandSub = nh.subscribe <std_msgs::Int16> ("/" + myNodeName + "/cmd", 1, &PubSubManager::cmdCallback, &psm);

	ros::spin();


	return EXIT_SUCCESS;
}
