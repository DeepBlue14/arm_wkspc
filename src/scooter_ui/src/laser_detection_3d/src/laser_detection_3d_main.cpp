#include "laser_detection_3d/laser_detection_3d.h"

int main(int argc, char **argv)
{
	std::string nodeName = "laser_detector_3d";
	ros::init(argc, argv, nodeName.c_str());

	ROS_INFO("Starting %s\n", nodeName.c_str());

	ros::NodeHandle nh;
	LaserDetector ld(nh);

	//TODO will have to publish a laser point estimate here, with 3D values
	//Loop at 15hz
	ros::Rate r(15);
	while(ros::ok())
	{
		ros::spinOnce();
		ld.getLaserPoint();

		r.sleep();
	}

	return EXIT_SUCCESS;
}
