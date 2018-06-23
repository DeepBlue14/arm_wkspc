#ifndef LASER_DETECTION_3D_H
#define LASER_DETECTION_3D_H


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Point.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <cstdlib>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <cmath>

class LaserDetector
{
	private:
		ros::NodeHandle nh;
		ros::Publisher pointPub;
		ros::Publisher pointDbgPub;
		ros::Subscriber imgSub;
		ros::Subscriber pcdSub;
		ros::Subscriber commandSub;
		std::vector<geometry_msgs::Point> canidatePts;
		std::vector<std::vector <cv::Point> > detectMotion();
		std::vector<std::vector <cv::Point> > detectBrightness();
		std::vector<std::vector <cv::Point> > checkColor(std::vector<std::vector <cv::Point> > contourList, int color, int maxDistance);
		std::vector<std::vector <cv::Point> > checkSize(std::vector<std::vector <cv::Point> > contourList, int maxSize);
		std::vector<std::vector <cv::Point> > checkMatches(std::vector<std::vector <cv::Point> > listA, std::vector<std::vector <cv::Point> > listB, float range);
		void checkDistance();

		//The frame the cloud should be converted to
		std::string frame;

		//Booleans for keeping track of what data we have
		bool havePointCloud;
		bool havePrevFrame;
		bool transform_is_init;
		bool haveAllData();
		//Our data, stored as it arrives
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		cv::Mat currentImage;
		cv::Mat prevImage;
		cv::Mat hsvImage;
		tf::TransformListener tf_l;
		tf::StampedTransform transform;

		//stores the selected point
		pcl::PointXYZRGB selectedPoint;

		//Debugging functions
		//RNG rng(12345);
		void drawAllContours(std::vector<std::vector <cv::Point> > contourList, std::string winName, cv::Scalar color);
		void showImg(cv::Mat img, std::string winName);

	public:
		void imgCallback(const sensor_msgs::ImageConstPtr& img);
		void pcdCallback(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		void cmdCallback(std_msgs::Int16 command);
	    ros::Publisher* getPublisher();
	    void getLaserPoint();
	    LaserDetector(ros::NodeHandle& nh);
	    ~LaserDetector() {};
};

#endif //LASER_DETECTION_3D_H
