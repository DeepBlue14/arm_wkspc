/* Find all the point cloud topics, and the transforms between them.
 * For all cloud topics that have transforms between them,
 * 1. Range filter the cloud
 * 2. Voxelize the cloud to save time on future operations
 * 3. Pairwise, transform cloud A to the frame of the cloud B (using the guess from the real-world measurements)
 * 4. Pairwise, apply ICP with normals & curvature to cloud A
 * 5. Combine clouds
 * 6. Publish a single, unified cloud.
 */

#include <cloud_registration/cloud_registration.h>

CloudMasher::CloudMasher(ros::NodeHandle& nh)
{
	//Set up our publisher for the registered clouds
	pub = nh.advertise < pcl::PointCloud<pcl::PointXYZRGB> > ("unified_cloud", 1);
	//debug_pub = nh.advertise < pcl::PointCloud<pcl::PointXYZRGB> > ("debug_cloud", 1);
	//debug_pub_2 = nh.advertise < pcl::PointCloud<pcl::PointXYZRGB> > ("debug_cloud_2", 1);
}

void CloudMasher::pointCloudCallback(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	//ROS_INFO("Got Cloud");

	//Filter the cloud by distance
	pcl::PassThrough < pcl::PointXYZRGB > pass;

	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.4);
	pass.setInputCloud(cloud);
	pass.filter(*cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-1.3, 1.3);
	pass.setInputCloud(cloud);
	pass.filter(*cloud);

	//Voxelize the point cloud, from Andreas's twopcd6.cpp
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vox(new pcl::PointCloud<
			pcl::PointXYZRGB>); //Cons of Ptr takes thing to point to, NULL otherwise
	pcl::VoxelGrid < pcl::PointXYZRGB > vox;
	vox.setInputCloud(cloud);
	vox.setLeafSize(0.005f, 0.005f, 0.005f);
	vox.filter(*cloud_vox);
	//Store this point cloud in the map of all point clouds, overwriting old ones
	clouds[cloud->header.frame_id] = cloud_vox;

	if (clouds.size() == 1)
	{
		//Only have one cloud, it is the one we just got, so publish that
		ROS_WARN("Only have one point cloud, publishing that");
		pub.publish(cloud);
	}
	else if (clouds.size() > 1)
	{
		//ROS_INFO("Have %lu clouds", clouds.size());
		//Have multiple clouds, so need to register them before publishing the unified cloud
		//This attempts to accumulate clouds in one big cloud, buy merging the first and second cloud, then merging the third cloud, etc.
		//So the end result is the first cloud and everything that can be transformed into its frame.

		for (std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::iterator from_cloud = (++clouds.begin());
				from_cloud != clouds.end(); from_cloud++)
		{
			//Try to get a transform from the first cloud to the next
			tf::StampedTransform transform;

			//Wait up to two seconds for transform, if you don't get it, continue
			if (tf_l.waitForTransform(clouds.begin()->first, from_cloud->first, ros::Time(0), ros::Duration(2.0)))
			{
				tf_l.lookupTransform(clouds.begin()->first, from_cloud->first, ros::Time(0), transform);
			}
			else
			{
				ROS_WARN("No transform from %s to %s arrived in 2 seconds", clouds.begin()->first.c_str(), from_cloud->first.c_str());
				continue;
			}

			//We didn't fail to get the transform, so use it
			//ROS_INFO("Got a transform from %s to %s", clouds.begin()->first.c_str(), from_cloud->first.c_str());
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<
					pcl::PointXYZRGB>);
			//Args are frame to transform to, input, output, and transform listener to use
			//So after this, temp_cloud is the cloud the iterator points to, transformed into the frame of the first cloud
			pcl_ros::transformPointCloud(clouds.begin()->first, *(from_cloud->second), *(temp_cloud), tf_l);
			if (temp_cloud->points.size() == 0)
			{
				ROS_WARN("Empty cloud after transformation.");
				continue;
			}

			//Estimate normals for first and new/temp cloud
			pcl::PointCloud<pcl::PointNormal>::Ptr begin_norm(new pcl::PointCloud<
					pcl::PointNormal>);
			pcl::PointCloud<pcl::PointNormal>::Ptr temp_norm(new pcl::PointCloud<
					pcl::PointNormal>);

			pcl::NormalEstimation < pcl::PointXYZRGB, pcl::PointNormal > norm_est;
			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<
					pcl::PointXYZRGB>);
			norm_est.setSearchMethod(tree);
			norm_est.setKSearch(30);

			norm_est.setInputCloud(clouds.begin()->second);
			norm_est.compute(*begin_norm);
			pcl::copyPointCloud(*(clouds.begin()->second), *begin_norm);

			norm_est.setInputCloud(temp_cloud);
			norm_est.compute(*temp_norm);
			pcl::copyPointCloud(*temp_cloud, *temp_norm);

			//Compute curvatures
			MyPointRepresentation point_representation;
			//Weight the 'curvature' dimension so that it is balanced against x, y, and z
			float alpha[4] =
			{ 1.0, 1.0, 1.0, 1.0 };
			point_representation.setRescaleValues(alpha);

			//Set up for Iterative Closest Point (ICP)
			pcl::IterativeClosestPointNonLinear < pcl::PointNormal, pcl::PointNormal > reg;
			reg.setTransformationEpsilon(1e-6);
			//Set the maximum distance between two correspondences (src<->tgt)
			//Note: adjust this based on the size of your datasets
			//Andreas used 0.01, 0.013, 0.02, and 0.04, judging by his comments.
			//Graphing the registration fitness over 0.01-0.35 showed abrupt drops at ~0.03 and again at ~0.12
			reg.setMaxCorrespondenceDistance(0.04); //pretty decent
			//reg.setMaxCorrespondenceDistance(0.07); //OK
			//reg.setMaxCorrespondenceDistance(0.01); //bad
			//reg.setMaxCorrespondenceDistance(0.125); //Good, but not good enough

			//Set the point representation
			reg.setPointRepresentation(boost::make_shared<
					const MyPointRepresentation>(point_representation));
			//Inputs are the normal point clouds
			reg.setInputSource(temp_norm);
			reg.setInputTarget(begin_norm);

			temp_norm->header.frame_id = clouds.begin()->first;
			//Publish the untransformed image
			//debug_pub.publish(temp_norm);

			begin_norm->header.frame_id = clouds.begin()->first;
			//debug_pub.publish(begin_norm);

			//Align into the first point cloud
			pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = temp_norm;
			reg.align(*reg_result);

			//Calculate the final transformation
			Eigen::Matrix4f icpTransf = reg.getFinalTransformation();
			//Transform it
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_final_cloud(new pcl::PointCloud<
					pcl::PointXYZRGB>);
			//pcl::transformPointCloud(*temp_cloud, *icp_final_cloud, icpFinal);
			pcl::transformPointCloud(*temp_cloud, *icp_final_cloud, icpTransf);

			//Combine the point clouds
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_combined(new pcl::PointCloud<
					pcl::PointXYZRGB>);
			*cloud_combined = *icp_final_cloud + *(clouds.begin()->second);

			pub.publish(cloud_combined);

			// remove NANs
//			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_combined_nan(new pcl::PointCloud<
//					pcl::PointXYZRGB>);
//			std::vector<int> indices;
//			pcl::removeNaNFromPointCloud(*cloud_combined, *cloud_combined_nan, indices); //TODO, can I do this inline?

		}
	}
}

AutoSubscriber::AutoSubscriber(ros::NodeHandle& nh):registerer(nh),nodeHandle(nh)
{
	//All the magic is in the initializer list
}

void AutoSubscriber::subscribe_all(const ros::TimerEvent&)
{
	//Get a set of all of the available topics that are point clouds.
	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

	for (ros::master::V_TopicInfo::iterator it = master_topics.begin();
			it != master_topics.end(); it++)
	{
		const ros::master::TopicInfo& info = *it;

		//Looking for point clouds of the type sensor_msgs/PointCloud2
		//TODO This checking by constant strings feels hacky, but it works for now
		if (info.datatype == "sensor_msgs/PointCloud2")
		{
			//Only use registered point clouds
			if (info.name.find("registered") != std::string::npos)
			{
				//If we haven't subscribed to it yet
				if (subs.find(info.name) == subs.end())
				{
					ROS_INFO("Subscribed to %s", info.name.c_str());
					subs[info.name] = nodeHandle.subscribe < pcl::PointCloud < pcl::PointXYZRGB > ::Ptr > (info.name, 1, &CloudMasher::pointCloudCallback, &registerer);
				}
			}
		}
	}

	//Subscribe to all the point cloud topics, sticking them in a big set to keep them around
//	std::set < ros::Subscriber > subs;
//	for (std::set<std::string>::iterator topic = point_cloud_topics.begin();
//			topic != point_cloud_topics.end(); topic++)
//	{
//		subs.insert(nh.subscribe < pcl::PointCloud < pcl::PointXYZRGB > ::Ptr > ((*topic), 1, &CloudMasher::pointCloudCallback, &registerer));
//		ROS_INFO("Subscribed to %s", (*topic).c_str());
//
//	}
}


CloudProxy::CloudProxy(ros::NodeHandle& node, std::string topic):_nh(node)
{
	_topic = topic;
}

void CloudProxy::pointCloudCallback(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	//Store the cloud, and increment a counter of how many clouds we've received
	_cloud = cloud;
	_cloudCount++;
	//The counter is used to throw away the first cloud, as it sometimes has noise
	if(_cloudCount > 3)
	{
		_hasCloud = true;
	}
}

void CloudProxy::begin()
{
	//Subscribe to topic
	_hasCloud = false;
	_cloudCount = 0;
	_sub = _nh.subscribe < pcl::PointCloud < pcl::PointXYZRGB > ::Ptr > (_topic, 5, &CloudProxy::pointCloudCallback, this);
}

void CloudProxy::end()
{
	//Unsubscribe from topic
	_sub.shutdown();
	//Just in case, although begin() should take care of this
	_hasCloud = false;
	_cloudCount = 0;
}

bool CloudProxy::hasCloud()
{
	return _hasCloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudProxy::getCloud()
{
	return _cloud;
}

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  exit(0);
}

int main(int argc, char **argv)
{
	//Start up my node
	ros::init(argc, argv, "cloud_registerer");
	ROS_INFO("Starting the cloud mashing machine!");

	ros::NodeHandle nh;

	/* Custom sigint handler because we're not just doing ros::Spin(), so it seems to ignore 
	 * the ^C signal consistently. 
	 */
	signal(SIGINT, mySigintHandler);
	/* Subscribing to both cameras at once results in a lot of NaN points due to interference 
	 * between the two point clouds. This should subscribe to one camera until two point clouds 
	 * arrive, subscribe to the other camera until two point clouds arrive, 
	 * combine the point clouds, and publish the result. 
	 * It waits for the second point cloud because the first point cloud from the camera sometimes
	 * has some noise in it. 
	 */
	ros::Publisher pub;	
	pub = nh.advertise < pcl::PointCloud<pcl::PointXYZRGB> > ("unifier/unified_cloud", 1);
	//Republish all the clouds, for the laser detector
	ros::Publisher cloud1pub = nh.advertise < pcl::PointCloud<pcl::PointXYZRGB> > ("unifier/cloud1", 1);
	ros::Publisher cloud2pub = nh.advertise < pcl::PointCloud<pcl::PointXYZRGB> > ("unifier/cloud2", 1);
	
	CloudProxy cp1(nh, "/rgbd_cam_1/depth_registered/points");
	CloudProxy cp2(nh, "/rgbd_cam_2/depth_registered/points");

	tf::TransformListener tf_l;
	tf::StampedTransform transform;
		
	while(ros::ok())
	{
		//Don't do anything unless someone's around to see it
		if(pub.getNumSubscribers() > 0)
		{ 
			//Subscribe to first camera and wait for a cloud
			ROS_INFO("Getting first cloud");
			cp1.begin();

			//Debugging counter
//			int spinCount = 0;
			while(!cp1.hasCloud())
			{
				ros::spinOnce();
				ros::Duration(0.1).sleep();

//				spinCount += 1;
			}
//			ROS_INFO("Got first cloud after %i spins", spinCount);
			cloud1pub.publish(cp1.getCloud());
			cp1.end();

			//Subscribe to second camera and wait for it to get a cloud
			ROS_INFO("Getting second cloud");
//			spinCount = 0;
			cp2.begin();
			while(!cp2.hasCloud())
			{
//				spinCount += 1;
				ros::spinOnce();
				ros::Duration(0.1).sleep();
			}
//			ROS_INFO("Got second cloud after %i spins", spinCount);
			cloud2pub.publish(cp2.getCloud());
			cp2.end();

			//Combine clouds
//			ROS_INFO("About to combine the clouds");
			
			// Stupid first version, get transform from the first cloud to the second 
			if (tf_l.waitForTransform(cp1.getCloud()->header.frame_id, cp2.getCloud()->header.frame_id, ros::Time(0), ros::Duration(2.0)))
			{
				tf_l.lookupTransform(cp1.getCloud()->header.frame_id, cp2.getCloud()->header.frame_id, ros::Time(0), transform);
			
//				ROS_INFO("Got transform");
				
				//Create a temp cloud to hold the concatenated results
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
				temp_cloud->header.frame_id = cp2.getCloud()->header.frame_id;

//				ROS_INFO("Created temp cloud");
				//Args are frame to transform to, input, output, and transform listener to use
				//So after this, temp_cloud is the cloud the iterator points to, transformed into the frame of the first cloud
				pcl_ros::transformPointCloud(cp1.getCloud()->header.frame_id, *cp2.getCloud(), *(temp_cloud), tf_l);

//				ROS_INFO("Did transform");
				//Concatenation operation may be really heavy, maybe voxelize first?
				*(temp_cloud) += *(cp1.getCloud());

//				ROS_INFO("Concatenated");
				//Publish result
				//ROS_INFO("About to publish result");
				pub.publish(temp_cloud);
//				ROS_INFO("Published");

			}
			else
			{
				ROS_WARN("No transform from %s to %s arrived in 2 seconds, retrying", cp1.getCloud()->header.frame_id.c_str(), cp2.getCloud()->header.frame_id.c_str());
			}
		}	
	}

	return EXIT_SUCCESS;
}
