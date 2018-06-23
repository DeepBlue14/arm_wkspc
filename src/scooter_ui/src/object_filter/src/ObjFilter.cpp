#include "object_filter/ObjFilter.h"

ObjectFilter::ObjectFilter() {
	//Initially, we don't have any data
	havePointCloud = haveLaserPoint = transform_is_init = false;
	//Initialize the transform from the point cloud to base
	frame = "/base";
}

ros::Publisher* ObjectFilter::getPointPublisher() {
	return &point3D_pub;
}

bool ObjectFilter::haveAllData() {
	//Only ready if we have everything
	//~ if (!haveLaserPoint) {
		//~ ROS_WARN("Cloud filter has no laser point");
	//~ } else if (!havePointCloud) {
		//~ ROS_WARN("Cloud filter has no point cloud");
	//~ }

	return (havePointCloud && haveLaserPoint);
}

void ObjectFilter::pointCallback(
		const geometry_msgs::PointConstPtr& pixelPoint) {
	this->pixelPoint = pixelPoint;
	//Negative values mean we don't have a valid location for the laser point
	//At this point we don't care about the z location of the point, as we get that later
	if ((pixelPoint->x < 0) || (pixelPoint->y < 0)) {
		haveLaserPoint = false;
	} else {
		haveLaserPoint = true;
	}
}

void ObjectFilter::pointCloudCallback(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	if (!cloud->isOrganized())
	{
		ROS_WARN("Cloud filter got a disorganized or undense cloud, disregarding it");
		havePointCloud = false;
	}
	else
	{
		this->cloud = cloud;
		havePointCloud = true;
	}
}

void ObjectFilter::clearState(){
	haveLaserPoint = havePointCloud = transform_is_init = false;
}

void ObjectFilter::filterCloud() {
	ROS_INFO("Getting laser point in 3D space");
	//Transform point cloud to base frame
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	temp_cloud->header.frame_id = frame;
	while (!transform_is_init) {
		//Wait up to two seconds for transform, if you don't get it, continue
		ROS_INFO("Getting transform from %s to %s", cloud->header.frame_id.c_str(), frame.c_str());
		if (tf_l.waitForTransform(cloud->header.frame_id, frame, ros::Time(0), ros::Duration(2.0))) {
			tf_l.lookupTransform(cloud->header.frame_id, frame,	ros::Time(0), transform);
			transform_is_init = true;
		} else {
			ROS_WARN("No transform from %s to %s arrived in 2 seconds, retrying", cloud->header.frame_id.c_str(), frame.c_str());
			transform_is_init = false;
		}
	}
	pcl_ros::transformPointCloud(frame, *cloud, *(temp_cloud), tf_l);
	cloud = temp_cloud;

	//Get the 3D point associated with the laser location
  if(!cloud->isOrganized())
  {
    ROS_ERROR("Cloud became disorganized after arriving at this node");
  }
  else
  {    
    pcl::PointXYZRGB selectedPoint = cloud->at(pixelPoint->x,
        pixelPoint->y);
    
    // search for non-NAN point in small neighborhood
    if (isnan(selectedPoint.x))
    {
      ROS_INFO("original pt is nan!");
      int x = -2;
      int y = -2;
      
      for (int i=0; i < 25; i++)
      {
        if (pixelPoint->x + x >= 0 && pixelPoint->x + x < 640 && pixelPoint->y + y >= 0 && pixelPoint->y + y < 480)
        {
          selectedPoint = cloud->at(pixelPoint->x + x, pixelPoint->y + y);
          if (!isnan(selectedPoint.x))
            break;
        }
        x++;
        if (x == 3)
        {
          x = -2;
          y++;
        }
      }
      ROS_INFO_STREAM("pixelPoint:" << pixelPoint->x << ", " << pixelPoint->y 
        << " selectedPoint:" << selectedPoint.x << ", " << selectedPoint.y << ", " << selectedPoint.z);
    }

    //Get the xyz of the selectedPoint and put it into a ROS point message
    geometry_msgs::Point outputPoint;
    outputPoint.x = selectedPoint.x;
    outputPoint.y = selectedPoint.y;
    outputPoint.z = selectedPoint.z;
    point3D_pub.publish(outputPoint);
  }
}

