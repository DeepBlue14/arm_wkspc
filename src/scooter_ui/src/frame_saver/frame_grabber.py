
#Get exactly one point cloud from a primesense device, and the corresponding RGB image

import sys
import rospy
from sensor_msgs.point_cloud2 import PointCloud2
from sensor_msgs.msg import Image
import cv
from cv_bridge import CvBridge
#import pcl

class receiver:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.img_callback)
        self.pcl_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, self.pcl_callback)
        self.img_saved = False
        self.pcl_saved = False
        
    def img_callback(self, data):
        if not self.img_saved:
            #Convert from a ROS message to an openCV iamge and use OpenCV's image writing to save it
            image = CvBridge().imgmsg_to_cv(data, desired_encoding="bgr8")
            cv.SaveImage("./cam_image.png", image)
            #TODO may have to keep track of the latest image, and save the one 
            #whose timestamp most closely matches the point cloud 
            self.img_saved = True
        else:
            #Unregister the image subscriber so we don't get more images
            self.image_sub.unregister()
            if self.pcl_saved:
                self.cleanup()
            
    def pcl_callback(self, data):
        if not self.pcl_saved:
            print("Received {0} data points in frame {1}".format(
                     data.width * data.height, data.header.frame_id)) #, pcl.getFieldsList(data))
            # Unfortunately, there's no way to save the point cloud to a PCD easily, or
            # that is likely to be correct (I could write my own PCD writer). There are python bindings
            # for PCL, but they can't be installed at the same time as Hydro's openni stuff. 
            self.pcl_saved = True
        else:
            #Stop listening for point clouds
            self.pcl_sub.unregister()
            if self.img_saved:
                self.cleanup()
                
    def cleanup(self):
        rospy.signal_shutdown("Saved both images, shutting down now.")
        

if __name__ == '__main__':
    #TODO probably want to load a prefix for the files here
    rospy.init_node('my_node_name', anonymous=True)
    r = receiver()
    rospy.spin()