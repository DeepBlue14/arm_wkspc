#!/usr/bin/python
# [db] 2013.11.14
import roslib
roslib.load_manifest("qrcode_detection")
import rospy
from sensor_msgs.msg import Image
import time
from qrcode_detection.msg import QRCode, QRCodes
from geometry_msgs.msg import Point, Pose,PoseStamped
import message_filters
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import numpy as np
import copy
import sys
import math
import time

#Removed after ROS Fuerte, use the local copy
#from python_msg_conversions import pointclouds
import pointclouds

# Import Libraries for working with zbar
import zbar
import Image as img
import MyFilter
from PointCloudTools import create_cloud_xyz,read_cloud_xyz, read_points, read_offset, read_offset_and_alternatives

# Import Visualization Stuff
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion,quaternion_about_axis, quaternion_from_euler

import tf

# Debug options
testmode = False
debug = True
WEBCAM = False
KINECT = True


# TODO: This could be much improved by 

def BuildMarker(color,x,y,z,quat,namespace,frame):
    marker = Marker()
    marker.header.frame_id = frame
    marker.ns = namespace
    marker.id = 5
    marker.type = 0
    marker.action = 0
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.03
    if color == "red":
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    elif color == "blue":
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
    marker.color.a = 1.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]
    return marker

# Helper function that returns two points pa and pb. pa is the point on line p1p2 that is closest to p3p4 and pb is the point on p3p4 closest to p1p2
def closest_points(p1, p2, p3, p4):
    # Helper function to calculate a relevant number
    def d(m,n,o,p):
        return (m[0]-n[0])*(o[0]-p[0]) + (m[1]-n[1])*(o[1]-p[1]) + (m[2]-n[2])*(o[2]-p[2]);
    pa = np.array([0.0,0.0,0.0])
    pb = np.array([0.0,0.0,0.0])
    mua = ( d(p1,p3,p4,p3)*d(p4,p3,p2,p1) - d(p1,p3,p2,p1)*d(p4,p3,p4,p3) ) / ( d(p2,p1,p2,p1)*d(p4,p3,p4,p3) - d(p4,p3,p2,p1)*d(p4,p3,p2,p1) )
    mub = ( d(p1,p3,p4,p3) + mua*d(p4,p3,p2,p1) ) / d(p4,p3,p4,p3)
    pa = p1 + mua * (p2 - p1)
    pb = p3 + mub * (p4 - p3)
    return pa, pb
def magnitude(vector):
    return np.sqrt(np.vdot(vector,vector))
def normalize(vector):
    return vector / magnitude(vector)
def midpoint(p1, p2):
    return np.array([(p1[0]+p2[0])/2.0,(p1[1]+p2[1])/2.0,(p1[2]+p2[2])/2.0])

class QRCodeDetector(object):
    def __init__(self,codepub):
        self.__blip = "|"
        self.publisher = codepub
        self.synchronizer = None
        self.lastkinectimage = [None,None,None]
        self.bufferindex = 0
        self.tfl = tf.TransformListener()
        
    def spinner(self):
        p = { "|": "/",
              "/" : "-",
              "-" : "\\",
              "\\" : "|"}
        self.__blip = p[self.__blip]
        print "\b\b"+self.__blip,
        sys.stdout.flush()

    def kinectdepth(self,points_msg):
        self.lastkinectimage[self.bufferindex]=points_msg
        self.bufferindex=(self.bufferindex+1)%3

    def kinectimage(self, img_msg):
        if (self.lastkinectimage[self.bufferindex]==None):
            return
        self.kinectcallback(img_msg, self.lastkinectimage[self.bufferindex])
        self.lastkinectimage[self.bufferindex]=None

    def kinectcallback(self,img_msg,points_msg):
#        self.spinner()
        scanner = zbar.ImageScanner()
        scanner.parse_config('enable')
        pil = img.fromstring('RGB',(img_msg.width,img_msg.height),img_msg.data,'raw')
        pil = pil.convert("L")
        width,height = pil.size
        raw = pil.tostring()
        image = zbar.Image(width,height, "Y800", raw)
        scanner.scan(image)

        #Transform point cloud to the RGB frame
        #Doesn't work, transformPointCloud doesn't suport PC2
        #points_msg = self.tfl.transformPointCloud("rgbd_cam_1_rgb_frame", points_msg)
        
        #NUMPY POINT CLOUD ZOOM ZOOM!
        temp_arr = pointclouds.pointcloud2_to_array(points_msg)
        new_dtype = []
        for field_name in temp_arr.dtype.names:
            field_type, field_offset = temp_arr.dtype.fields[field_name]
            if not field_name == 'rgb':
                new_dtype.append((field_name, field_type))
        cloud_arr = np.zeros(temp_arr.shape, new_dtype)
        cloud_arr['x'] = temp_arr['x']
        cloud_arr['y'] = temp_arr['y']
        cloud_arr['z'] = temp_arr['z']

        avg_yaw=0.0
        avg_pitch=0.0
        avg_xyz=(0.0,0.0,0.0)
        avg_count=0        
        
        codes=QRCodes()
        for symbol in image:
            #print "\b\b "
            #print "decoded", symbol.type, " time '%s.%s'"%(str(img_msg.header.stamp.secs),str(img_msg.header.stamp.nsecs))," at ",time.time()

            # Print Time Stamps
            #print "i:",img_msg.header.stamp.secs, img_msg.header.stamp.nsecs
            #print "p:",points_msg.header.stamp.secs, points_msg.header.stamp.nsecs

            # Don't try to decode anything that is not recognized as a QRCode
            if not str(symbol.type) == "QRCODE":
                continue

            locs = list()
            for x in symbol.location:
                locs.append((x[0]*640/width,x[1]*480/height))

            #print "Detected '%s' @ %s.%s"%(symbol.data,str(img_msg.header.stamp.secs),str(img_msg.header.stamp.nsecs))
            # Translate from 2D to 3D points
            found=0
            output=list()
            output_arr = np.zeros(cloud_arr.shape, cloud_arr.dtype)
                              
#            print "------"
            for pt in locs:
                #checks pixel at (x,y) and its neighbors to find one of them that's non-nan
                pts3d = read_offset_and_alternatives(cloud_arr,pt[0],pt[1],points_msg.width,points_msg.height)
                #print pt,"->", pts3d
                
                if pts3d==None:
                    #one of our 4 points was nan, as were its 4 adjacent pixels
                    break      
 #               print pt, " ", pts3d

                #received a valid point at (x,y) (or one of its neighbors)
                output.append(np.array((pts3d['x'],pts3d['y'],pts3d['z'])))
                output_arr[pt[0],pt[1]]=pts3d
                
#              print output
        # Output debug point cloud
            if len(output) > 0:
                output_cloud = pointclouds.array_to_pointcloud2(output_arr, stamp=rospy.Time.now(), frame_id=points_msg.header.frame_id)
                #print output_cloud
#                output_cloud = create_cloud_xyz(Header(frame_id=points_msg.header.frame_id), output)
                points_pub.publish(output_cloud) 

            # Decode a pose if we have 3D info for the whole thing
            if len(output) == 4:
                # these poses are all non-nan. bow chicka wow wow

                # Find the plane
                # TODO: Right now I do this with a crossproduct, using 3 of the 4 points.
                # It would be better to do some kind of least squares optimization

                # Calculate the orientation quaternion of the vector normal to the plane
                norm = np.cross(output[1]-output[0],output[3]-output[0])
                norm = normalize(norm)
                yaw = math.atan2(norm[1],norm[0])
                pitch = -math.atan2(norm[2], math.sqrt(norm[0]*norm[0]+norm[1]*norm[1]))
                quat = quaternion_from_euler(0,pitch,yaw)

                # Determine the center of the plane
                midline_pts = closest_points(output[0],output[2],output[1],output[3])
                center = midpoint(midline_pts[0],midline_pts[1]) 

                # Get the "Front" of the QR Code, between the top two points
                front = midpoint(output[0],output[3]) 
                forward = normalize(front - center)

                marker = BuildMarker("red",center[0],center[1],center[2],quat,"norm",points_msg.header.frame_id)
                marker1_pub.publish(marker)

                # Determine quaterion orientation for forward vector
                # NOTE: The foward vector and vector normal to the qrcode are not necessarily square!
                yaw = math.atan2(forward[1],forward[0])
                pitch = -math.atan2(forward[2], math.sqrt(forward[0]*forward[0]+forward[1]*forward[1]))
                quat = quaternion_from_euler(0,pitch,yaw)

                marker = BuildMarker("blue",center[0],center[1],center[2],quat,"norm",points_msg.header.frame_id)
                marker2_pub.publish(marker)

                # Publish QRCode and Pose
                pose = Pose()
                pose.position.x = center[0]
                pose.position.y = center[1]
                pose.position.z = center[2]
                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]
                code = QRCode()
                a,b,c,d = locs[0], locs[1], locs[2], locs[3]
                code.location.append(Point(x=a[0],y=a[1]))
                code.location.append(Point(x=b[0],y=b[1]))
                code.location.append(Point(x=c[0],y=c[1]))
                code.location.append(Point(x=d[0],y=d[1]))
                code.data = symbol.data
                #code.image = img_msg
                code.pose = pose
                codes.codes.append(code)
                    
                # Publish the pose by itself
                msg = PoseStamped()
                msg.header.frame_id = points_msg.header.frame_id
                msg.pose = pose
                pose_pub.publish(msg)

        if len(codes.codes) > 0:
            qrcode_pub.publish(codes)
            
        del(image)


    def imagecallback(self,msg):
        if debug:
            print ""
            print ".",
        scanner = zbar.ImageScanner()
        scanner.parse_config('enable')
        pil = img.fromstring('RGB',(msg.width,msg.height),msg.data,'raw')
        pil = pil.convert("L")
        width,height = pil.size
        raw = pil.tostring()
        image = zbar.Image(width,height, "Y800", raw)
        scanner.scan(image)
        for symbol in image:
            if debug:
                print "decoded", symbol.type, " symbol '%s'"%symbol.data
                print symbol.location
            # Send QRCode
            code = QRCode()
            a,b,c,d = symbol.location[0], symbol.location[1], symbol.location[2], symbol.location[3]
            code.location.append(Point(x=a[0],y=a[1]))
            code.location.append(Point(x=b[0],y=b[1]))
            code.location.append(Point(x=c[0],y=c[1]))
            code.location.append(Point(x=d[0],y=d[1]))
            code.data = symbol.data
            code.image = msg
            self.publisher.publish(code)

        del(image)
if __name__ == '__main__':
    rospy.init_node('qrcode_detection')
    
    # Get Ros topics
    image_topic = rospy.get_param('~image_topic','/pandaexpress/rgb/image_rect_color')#'/gscam/image_raw')
    points_topic = rospy.get_param('~points_topic','/pandaexpress/depth_registered/points')
    code_topic = rospy.get_param('~qrcode_topic','/qrcode')
    debug_topic = rospy.get_param('~debug_topic','/debug_points')

    print "qrcode_detection: Subscribing to '%s'"%image_topic
    print "qrcode_detection: Subscribing to '%s'"%points_topic

    # Output Topics
    qrcode_pub = rospy.Publisher(code_topic,QRCodes, queue_size = 1)

    detector = QRCodeDetector(qrcode_pub)
    # Detect from Image
    if WEBCAM:
        rospy.Subscriber(image_topic,Image,(lambda x: detector.imagecallback(x)))

    # Detect from kinect data - synchronize frames
    print "qrcode_detection: connecting to kinect"
    # Input Data
    #image_sub = message_filters.Subscriber(image_topic,Image)
    #points_sub = message_filters.Subscriber(points_topic,PointCloud2)
    #image_sub = MyFilter.ThrottledSubscriber(image_topic,Image,10)
    #points_sub = MyFilter.ThrottledSubscriber(points_topic,PointCloud2,10)
    image_sub = rospy.Subscriber(image_topic,Image,(lambda x: detector.kinectimage(x)))
    points_sub = rospy.Subscriber(points_topic,PointCloud2,(lambda x: detector.kinectdepth(x)))
    
    # Dirty Hack to Set the Queue Size of the Filter Subscribers
    #image_sub.sub.impl.set_queue_size(1)
    #points_sub.sub.impl.set_queue_size(1)
    # More Output
    points_pub = rospy.Publisher(debug_topic,PointCloud2, queue_size = 1)
    pose_pub = rospy.Publisher('/qrcode_pose',PoseStamped, queue_size = 1)

    # Debug Markers
    marker1_pub = rospy.Publisher('/qrcode_marker1',Marker, queue_size=5)
    marker2_pub = rospy.Publisher('/qrcode_marker2',Marker, queue_size=5)

    #ts = message_filters.TimeSynchronizer([image_sub,points_sub],100)
    #ts = MyFilter.MySynchronizer([image_sub,points_sub],50,1)
    #detector.synchronizer = ts
    #ts.registerCallback(lambda x,y: detector.kinectcallback(x,y))


    if testmode:
        # Test Mode - Send a known good test picture
        pub = rospy.Publisher(image_topic,Image)
        while not rospy.is_shutdown():
            pil = img.open("test.png").convert("RGB")
            msg = Image()
            msg.height = pil.size[1]
            msg.width = pil.size[0]
            msg.encoding = 'rgb8'
            msg.data = pil.tostring('raw')
            pub.publish(msg)
            time.sleep(1)
    else:
        rospy.spin()
