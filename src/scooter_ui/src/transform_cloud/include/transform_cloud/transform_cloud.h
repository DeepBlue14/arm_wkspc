#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <tf/transform_listener.h>
#include <pcl/filters/filter.h> //for range filtering
#include <pcl/filters/passthrough.h> //also for range filtering
#include <pcl/filters/voxel_grid.h> //For decimation
class CloudTransformer
{
	private:
		tf::TransformListener tf_l;
		tf::StampedTransform transform;
		bool transform_is_init;
		ros::NodeHandle& nh;
		ros::Publisher pub;
		ros::Subscriber sub;
		std::string to, from, frame;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	public:
		CloudTransformer(std::string fromTopic, std::string toTopic, std::string toFrame, ros::NodeHandle& nh);
		void init();
		void update();
		void cloudCallback(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		~CloudTransformer(){}; //Doesn't do anything (except prevent a linker error)
};
