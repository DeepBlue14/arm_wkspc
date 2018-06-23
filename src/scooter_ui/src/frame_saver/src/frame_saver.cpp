#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class MatchSaver
{
	protected:
		ros::NodeHandle n;
		ros::Subscriber imgSub, pclSub;

	private:

		sensor_msgs::ImageConstPtr mostRecentMsg;
		pcl::PCLPointCloud2::ConstPtr mostRecentCloud;
		bool haveImage;
		bool havePCL;

		void saveImage(std::string outPrefix)
		{
			cv::Mat image;
			try
			{
				//Convert, Primesense images are BGR8
				image = cv_bridge::toCvShare(mostRecentMsg, "bgr8")->image;
			} catch (cv_bridge::Exception)
			{
				ROS_ERROR("Unable to convert %s image to bgr8", mostRecentMsg->encoding.c_str());
				return;
			}
			if (!image.empty())
			{
				//Create the filename and write to that file
				std::stringstream fName;
				fName << outPrefix << ".png";
				cv::imwrite(fName.str().c_str(), image);
			}
			else
			{
				ROS_WARN("No data in image message.");
			}
		}

		void saveCloud(std::string outPrefix)
		{
			if ((mostRecentCloud->width * mostRecentCloud->height) == 0)
			{
				//Some axis was 0, so no data
				ROS_WARN("No data in cloud message (width or height was 0).");
			}
			else
			{
				//Create the file name by adding an appropriate extension
				std::stringstream fName;
				fName << outPrefix << ".pcd";

				//Save to that file
				pcl::PCDWriter writer;
				writer.writeASCII(fName.str(), *mostRecentCloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), 8);
			}
		}

		void saveData()
		{
			//Make up a file name prefix, currently using the timestamp of the cloud
			std::stringstream fNamePrefix;
			fNamePrefix << mostRecentCloud->header.stamp;

			saveCloud(fNamePrefix.str());
			saveImage(fNamePrefix.str());
			
			//Saved both a file and an image, so shut this node down. 
			ros::shutdown();
		}

		//Only save the most recent image. Images are published at a much higher rate than 
		//point clouds, so an image will almost certainly arrive before a point cloud. 
		void imgReceived(const sensor_msgs::ImageConstPtr& image_msg)
		{
			mostRecentMsg = image_msg;
			haveImage = true;
			if (havePCL && haveImage)
			{
				saveData();
			}
		}

		//Pretty much exactly the same as receiving an image. Only keep track of the most recent cloud
		void pclReceived(const pcl::PCLPointCloud2::ConstPtr& cloud)
		{
			mostRecentCloud = cloud;
			havePCL = true;
			if (havePCL && haveImage)
			{
				saveData();
			}
		}

	public:
		MatchSaver()
		{
			//Don't have either set of data yet
			haveImage = havePCL = false;
			//Subscribe to get them
			imgSub = n.subscribe("/rgbd_cam_1/rgb/image_raw", 1, &MatchSaver::imgReceived, this);
			pclSub = n.subscribe("/rgbd_cam_1/depth_registered/points", 1, &MatchSaver::pclReceived, this);
		}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener", ros::init_options::AnonymousName);
	MatchSaver ms;
	ros::spin();

	return 0;
}
