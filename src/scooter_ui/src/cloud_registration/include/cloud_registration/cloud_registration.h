#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
//Added to attempt online cloud registration
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/visualization/cloud_viewer.h>

class CloudMasher
{
	private:
		std::map< std::string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clouds;
		tf::TransformListener tf_l;
		ros::Publisher pub;
		//ros::Publisher debug_pub;
		//ros::Publisher debug_pub_2;
	public:
		CloudMasher(ros::NodeHandle& node);
		void pointCloudCallback(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		~CloudMasher(){}; //Doesn't do anything (except prevent a linker error)
};

class CloudProxy
{
private:
	bool _hasCloud;
	int _cloudCount;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud;
	ros::NodeHandle& _nh;
	ros::Subscriber _sub;
	std::string _topic;

public:
	CloudProxy(ros::NodeHandle& node, std::string topicName);
	void pointCloudCallback(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	void begin();
	void end();
	bool hasCloud();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud();
	
~CloudProxy(){};
};


class AutoSubscriber
{
	private:
		std::map<std::string, ros::Subscriber> subs;
		CloudMasher registerer;
		ros::NodeHandle& nodeHandle;
	public:
		AutoSubscriber(ros::NodeHandle& node);
		void subscribe_all(const ros::TimerEvent&);
		~AutoSubscriber(){};
};

// Define a new point representation for < x, y, z, curvature >
// From Andreas's twopcd6.cpp
class MyPointRepresentation : public pcl::PointRepresentation <pcl::PointNormal>
{
  using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const pcl::PointNormal &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};
