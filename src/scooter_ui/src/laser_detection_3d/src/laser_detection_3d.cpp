#include "laser_detection_3d/laser_detection_3d.h"

LaserDetector::LaserDetector(ros::NodeHandle& nh)
{
	this->nh = nh;
	frame = "/base";
	havePrevFrame = havePointCloud = false;

	pointPub = nh.advertise<geometry_msgs::Point>("/laser_detector/point", 1);
    pointDbgPub = nh.advertise<geometry_msgs::PointStamped>("laser_detector/pointDbg", 1);
	//Subscribe the laser point detector to the image input
	//Subscribing to the image doesn't start the IR beam, so this is OK
	imgSub = nh.subscribe<sensor_msgs::Image>("/rgbd_cam_1/rgb/image_rect_color", 10, &LaserDetector::imgCallback, this);

	//Subscribe to the point cloud in order to find the laser point in 3d
	//Subscribing to the point cloud published by the camera causes interference between the cameras,
	//so this subscribes to the cloud republished by the cloud registration node
	pcdSub = nh.subscribe< pcl::PointCloud < pcl::PointXYZRGB > ::Ptr > ("/unifier/cloud1", 1, &LaserDetector::pcdCallback, this);

	//Subscriber for commands to unsubscribe the point cloud subscriber
	commandSub = nh.subscribe <std_msgs::Int16> ("/laser_detector/cmd", 1, &LaserDetector::cmdCallback, this);

}

void LaserDetector::cmdCallback(std_msgs::Int16 command)
{
	// if(command.data == 1)
	// {
	// 	//Subscribe to the topic
	// 	pcdSub = nh.subscribe< pcl::PointCloud < pcl::PointXYZRGB > ::Ptr > ("/rgbd_cam_1/depth/points", 1, &LaserDetector::pcdCallback, this);
	// }
	// else if(command.data == 0)
	// {
	// 	//Unsubscribe to stop beaming IR all over the place
	// 	pcdSub.shutdown();
	// 	havePointCloud = false;
	// }
	// else
	// {
	// 	ROS_WARN("Got invalid command %d", command.data);
	// }
	ROS_WARN("LaserDetector doesn't take commands anymore");
}

//Motion detection from James's code
std::vector<std::vector <cv::Point> > LaserDetector::detectMotion()
{
	std::vector<std::vector <cv::Point> > contours;

	int threshold = 30;//(int) (0.75 * maxVal);
	int blur = 5;
	if(havePrevFrame)
	{
		//Convert both images to grayscale
		cv::Mat grayImage1;
		cv::Mat grayImage2;
		cv::cvtColor(prevImage, grayImage1, cv::COLOR_BGR2GRAY);
		cv::cvtColor(currentImage, grayImage2, cv::COLOR_BGR2GRAY);

		//Subtract one from the other to get the difference between the frames
		cv::Mat differenceImage;
		cv::absdiff(grayImage1, grayImage2, differenceImage);
		//Threshold and blur to eliminate e.g. single-pixel camera noise
		cv::Mat thresholdImage;
		cv::threshold(differenceImage, thresholdImage, threshold, 255, cv::THRESH_BINARY);// | cv::THRESH_OTSU);
		cv::blur(thresholdImage, thresholdImage, cv::Size(blur, blur) );
		cv::threshold(thresholdImage, thresholdImage, threshold, 255, cv::THRESH_BINARY);

		//Find contours and store points as canidate locations
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(thresholdImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

	}else{
		ROS_WARN("No previous frame, so motion detection probably won't work");
	}

	return contours;
}

//Brightness checking from my laser detection code
std::vector<std::vector <cv::Point> >  LaserDetector::detectBrightness()
{
	//Operating on the HSV image
	//Split into H, S, V channels
	std::vector<cv::Mat> channels;
	cv::split(hsvImage, channels);

	//Get the average of the value channel
	cv::Scalar avgV = cv::mean(channels[2]);
	//While the average of the value channel is greater than 30% of the max possible value of the channel
	//Scalar is a 4-element array of doubles, in this case it would be HSVA, but I'm only working with V
	while(avgV[0] > (255 * 0.3))
	{
		//Subtract a small amount from the value channel and repeat
		cv::subtract(channels[2], 8, channels[2]);
		avgV = cv::mean(channels[2]);
	}

	//Put the HSV image back together, get its max value, and set the threshold to 95% of that
	cv::merge(channels, hsvImage);
	double maxVal;
	cv::minMaxIdx(hsvImage, NULL, &maxVal);
	int threshold = (int) (0.85 * maxVal);
	//ROS_WARN("Threshold for brightness is %i", threshold);
	//Threshold and blur V channel to eliminate e.g. single-pixel camera noise
	cv::Mat thresholdImage;
	cv::threshold(channels[2], thresholdImage, threshold, 255, cv::THRESH_BINARY);// | cv::THRESH_OTSU);
	//showImg(thresholdImage, "bright");
	//cv::blur(thresholdImage, thresholdImage, cv::Size(5, 5) );
	//cv::threshold(thresholdImage, thresholdImage, threshold, 255, cv::THRESH_BINARY);


	//Find contours and store points as canidate locations
	std::vector<cv::Vec4i> heirarchy;
	std::vector<std::vector <cv::Point> > contours;
	cv::findContours(thresholdImage, contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

	return contours;
}

//Color checking from James or translated from my laser detection attempt
std::vector<std::vector <cv::Point> > LaserDetector::checkColor(std::vector<std::vector <cv::Point> > contourList, int color, int maxDistance)
{
	//Given a list of contours, filter out all where the average color of the area around the contour is
	//too far from a given color
	std::vector<std::vector <cv::Point> > goodContours;
	std::vector<cv::Moments> mu(contourList.size() );
	for( int i = 0; i < contourList.size(); i++ )
	{
		mu[i] = cv::moments( contourList[i], false );
	}
	for( int i = 0; i< contourList.size(); i++ )
	{
		//Get the average color of a region around the contour
		//This is to ensure that we get some spill from the laser, which is colorful
		cv::Point2f center;
		float radius;
		cv::minEnclosingCircle(contourList[i], center, radius);
		//Get the average color of a region
		float startX = center.x;
		float startY = center.y;
		float width = 5;
		//Check that all the ROIs are inside the image
		if ((startX > 0) && (startY > 0) && ((startX + width) < currentImage.cols) && ((startY + width) < currentImage.rows)){
			cv::Mat roi = currentImage(cv::Rect(startX, startY, width, width));
			cv::Scalar meanColor = cv::mean(roi);
			//Set a mat to the average color, convert it to HSV, and get its hue
			cv::Mat colorSample = cv::Mat::zeros(cv::Size(5,5), CV_8UC3);
			colorSample.setTo(meanColor);
			cv::Mat colorHSV;
			cv::cvtColor(colorSample, colorHSV, CV_BGR2HSV);
			std::vector<cv::Mat> channels;
			cv::split(colorHSV, channels);

			//Green laser ends up 50 +/- about 8
			int hue = channels[0].at<uchar>(2,2);
			//TODO this might be wiggy with red (hues at both ends of the 0-180 range)
			if (abs(hue - color) < maxDistance)
			{
				goodContours.push_back(contourList[i]);
			}else{
				//ROS_WARN("Rejected contour %d with color %d", i, hue);
			}
		}
	}
	return goodContours;

}

//Contour size checking
//Contours that are too large are probably people or reflections
std::vector<std::vector <cv::Point> > LaserDetector::checkSize(std::vector<std::vector <cv::Point> > contourList, int maxSize)
{
	//Given a list of contours, filter out all contours below a minimum size
	std::vector<std::vector <cv::Point> > goodContours;
	std::vector<cv::Moments> mu(contourList.size() );
	for( int i = 0; i < contourList.size(); i++ )
	{
		mu[i] = cv::moments( contourList[i], false );
	}
	for( int i = 0; i< contourList.size(); i++ )
	{
		if(mu[i].m00 < maxSize)
		{
			goodContours.push_back(contourList[i]);
		}
		else
		{
			//ROS_INFO("Contour %d rejected with size %3.3f", i, mu[i].m00);
		}
	}
	return goodContours;
}

//Circular/oval dot checking?

//Check position of all canidate points relative to the camera origin
//This is to find which points are close, and attempt to ignore people in the scene
void LaserDetector::checkDistance()
{
	ROS_INFO("checkDistance()");

}

//Get the image from ROS and prepare it for processing
void LaserDetector::imgCallback(const sensor_msgs::ImageConstPtr& img)
{
	//Convert from image message to something opencv can work with
	cv_bridge::CvImagePtr cv_ptr;
	cv::Mat cvImage;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(img);
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what() );
	}

	cvImage = cv_ptr->image;

	//Convert bgr to rgb
	//TODO this because the above stuff gets us an RGB image. Can we just use CV_RGB2HSV below?
	int tmpColor;
	for(size_t a = 0; a < cvImage.rows; a++)
	{
		for(size_t b = 0; b < cvImage.cols; b++)
		{
			tmpColor = cvImage.at<cv::Vec3b>(a, b)[0];
			cvImage.at<cv::Vec3b>(a, b)[0] = cvImage.at<cv::Vec3b>(a, b)[2];
			cvImage.at<cv::Vec3b>(a, b)[2] = tmpColor;
		}
	}

	//Convert to hsv and store it
	cv::cvtColor(cvImage, hsvImage, CV_BGR2HSV);

	if (!havePrevFrame)
	{
		//prev and current both get incoming image, so motion detection won't work for the first frame
		cvImage.copyTo(prevImage);
		cvImage.copyTo(currentImage);
		havePrevFrame = true;
	}else{
		//move the previous current image to previous image
		currentImage.copyTo(prevImage);
		cvImage.copyTo(currentImage);
	}
}

void LaserDetector::showImg(cv::Mat img, std::string winName)
{
	cv::namedWindow(winName, CV_WINDOW_AUTOSIZE);
	cv::imshow(winName, img);
	cv::waitKey(3);
}

void LaserDetector::drawAllContours(std::vector<std::vector <cv::Point> > contourList, std::string winName, cv::Scalar color)
{
	//Don't try working on the cloud unless we actually have one
	if (haveAllData()){
		cv::Mat output = cv::Mat::zeros(hsvImage.size(), CV_8UC3);
		//get contour moments
		std::vector<cv::Moments> mu(contourList.size() );
		for( int i = 0; i < contourList.size(); i++ )
		{

		}

		for( int i = 0; i< contourList.size(); i++ )
		{
			//Draw the contour
			cv::drawContours(output, contourList, i, color, 2, 8);
			//get contour center
			if(mu[i].m00 == 0)
			{
				continue;
			}
			cv::Point2f contourCenter = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
			int x = (int) contourCenter.x;
			int y = (int) contourCenter.y;

			if ((0 <= x < cloud->width) && (0 <= y < cloud->height)){
				//Get the 3D point associated with the contour center
				//ROS_WARN("(%d, %d)", x, y);
				pcl::PointXYZRGB selectedPoint = cloud->at(x, y);
				//TODO if selectedPoint.z is NaN, deal with it
				//Put the text value of the z position on the contour
				char location[10];
				sprintf(location, "%3.3f", selectedPoint.z);
				cv::putText(output, location, contourCenter, cv::FONT_HERSHEY_SIMPLEX, 0.70, cv::Scalar(color[1], color[0], color[2]));
			}
		}
		cv::namedWindow(winName, CV_WINDOW_AUTOSIZE);
		cv::imshow(winName, output);
		cv::waitKey(3);
	}
}

void LaserDetector::pcdCallback(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	//We only operate on organized clouds, you can't 2d index a disorganized cloud
	if (!cloud->isOrganized())
	{
		ROS_WARN("Cloud filter got a disorganized cloud, disregarding it");
		havePointCloud = false;
	}
	else
	{
		this->cloud = cloud;
		havePointCloud = true;
	}
}

bool LaserDetector::haveAllData()
{
	//TODO extend as other data gets added
	return havePointCloud && havePrevFrame;
}

std::vector<std::vector <cv::Point> > LaserDetector::checkMatches(std::vector<std::vector <cv::Point> > listA, std::vector<std::vector <cv::Point> > listB, float range)
{
	std::vector<std::vector <cv::Point> > matches;
	//Brute force search, if all points in one contour are close to all points in another contour, add it to the list
	for( int i = 0; i< listA.size(); i++ )
	{

		cv::Moments mA = cv::moments( listA[i], false );
		for( int j = 0; j< listB.size(); j++ )
		{
			//Distance between centers of contours
			cv::Moments mB = cv::moments(listB[j], false);
			cv::Point2f centerA = cv::Point2f( mA.m10/mA.m00 , mA.m01/mA.m00 );
			cv::Point2f centerB = cv::Point2f( mB.m10/mB.m00 , mB.m01/mB.m00 );
			if (isnan(centerA.x) || isnan(centerA.y) || isnan(centerB.x) || isnan(centerB.y))
			{
				continue;
			}

			//Get the euclidian distance between the two centers
			float dist = sqrt(pow((centerA.x - centerB.x), 2) + pow((centerA.y - centerB.y), 2));
			//Check that the distance is less than the specified range
			if(dist < range)
			{
				matches.push_back(listB[j]);
				matches.push_back(listA[i]);
			}
		}
	}
	return matches;
}

//Tie everything together
void LaserDetector::getLaserPoint()
{
	if(haveAllData()){
		std::vector<std::vector <cv::Point> > motionContours;
		std::vector<std::vector <cv::Point> > brightContours;
		//Get the bright contours and moving contours
		motionContours = detectMotion();
		brightContours = detectBrightness();
		//Filter out all contours bigger than a fixed size
    //ROS_INFO("before cutting. m: %d b: %d", (int) motionContours.size(), (int) brightContours.size());
		brightContours = checkSize(brightContours, 100);
		motionContours = checkSize(motionContours, 200);
    //ROS_INFO("after cutting. m: %d b: %d", (int) motionContours.size(), (int) brightContours.size());
		std::vector<std::vector <cv::Point> > matchingContours = checkMatches(motionContours, brightContours, 12.0);
    
	    // If there are no matching contours, fall back to using the motion contours
	    if(matchingContours.size() == 0)
		{
	      matchingContours = motionContours;
	      ROS_WARN("No matching contours, falling back to %d motion contours", (int) motionContours.size());
	    }

		//At this point we have a (possibly empty) vector of contours
		//Find the highest one in the picture plane (laser approaches from above and reflects downward)
		//And return the x,y,z coordinates of that
		if(matchingContours.size() == 0)
		{
			ROS_ERROR("No contours available.");//" motion: %d brightness: %d selectedPoint: (%.3f, %.3f, %.3f)", (int) motionContours.size(), (int) brightContours.size(), selectedPoint.x, selectedPoint.y, selectedPoint.z);
			selectedPoint.x = selectedPoint.y = selectedPoint.z = NAN;
		}
		else{
			//X and Y locations of highest contour center
			int maxX = 0;
			int maxY = 0;

			//Find the uppermost contour location
			for( int i = 0; i< matchingContours.size(); i++ )
			{
				cv::Moments mu = cv::moments(matchingContours[i], false);
				cv::Point2f contourCenter = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
				if (isnan(contourCenter.x) || isnan(contourCenter.y))
				{
					continue;
				}

				//Image coordinates for getting 3D location are integer pixels
				int x = (int) contourCenter.x;
				int y = (int) contourCenter.y;

				//Update maxima if needed
				if (x > maxX)
				{
					maxX = x;
					maxY = y;
				}
			}

			//ROS_INFO_STREAM("maxX: " << maxX << " maxY: " << maxY);

			//Get the 3D point associated with the contour center
			if ((0 <= maxX < cloud->width) && (0 <= maxY < cloud->height)){
				selectedPoint = cloud->at(maxX, maxY);
				//Don't use points where the z coordinate is NaN (holes in the point cloud)
				if(isnan(selectedPoint.z))
				{
					//ROS_INFO("original pt is nan!");
					int x = -2;
					int y = -2;

					//Search a small area for for a non-NaN point 
					for (int i=0; i < 25; i++)
					{
						if (maxX + x >= 0 && maxX + x < 640 && maxY + y >= 0 && maxY + y < 480)
						{
							selectedPoint = cloud->at(maxX + x, maxY + y);
							if (!isnan(selectedPoint.z))
							{
								break;
							}
						}
						x++;
						if (x == 3)
						{
							x = -2;
							y++;
						}
					}
				}
				//ROS_WARN("(%.3f, %.3f, %.3f)", selectedPoint.x, selectedPoint.y, selectedPoint.z);
			}
			else{
				ROS_WARN("Center of highest contour is out of image!");
			}
		}
    
    	//Publish whatever we got, unless we got all NaNs
		if(!(isnan(selectedPoint.x) || isnan(selectedPoint.y) || isnan(selectedPoint.z)))
		{
			geometry_msgs::Point outputPoint;
			outputPoint.x = selectedPoint.x;
			outputPoint.y = selectedPoint.y;
			outputPoint.z = selectedPoint.z;
			pointPub.publish(outputPoint);

			geometry_msgs::PointStamped debugPoint;
			debugPoint.header.stamp = ros::Time::now();
			debugPoint.point = outputPoint;
			debugPoint.header.frame_id = "rgbd_cam_1_depth_optical_frame";
			pointDbgPub.publish(debugPoint);
		}
		else
		{
			ROS_WARN("Point had NaN value, not publishing it");
		}
	}
}

