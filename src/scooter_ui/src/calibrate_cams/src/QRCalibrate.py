#!/usr/bin/python
from __future__ import division

import rospy
import tf

import sys
import math
import random
import numpy as np
import pprint as pp
import copy

import baxter_interface
from baxter_core_msgs.msg import JointCommand, EndpointState

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header
from sensor_msgs.msg import Image

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


from grasp_selection.srv import *

import tf.transformations
from tf import TransformListener

#Image processing stuff
import zbar
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy

from visualization_msgs.msg import Marker

from pprint import pprint

import threading

end_marker_pub = rospy.Publisher('/endpoint_marker', Marker, queue_size=5)
qr_marker_pub = rospy.Publisher('/qr_marker', Marker, queue_size=5)

def BuildMarker(pose, frame, type = 1):
    marker = Marker()
    marker.header.frame_id = frame
    marker.ns = "Endpoint"
    marker.id = 5
    marker.type = type #0 is an arrow, 1 is a cube, 2 is a sphere
    marker.action = 0
    marker.scale.x = 0.02
    marker.scale.y = 0.02
    marker.scale.z = 0.02
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.pose.position.x = pose.position.x
    marker.pose.position.y = pose.position.y
    marker.pose.position.z = pose.position.z
    marker.pose.orientation.x = pose.orientation.x
    marker.pose.orientation.y = pose.orientation.y
    marker.pose.orientation.z = pose.orientation.z
    marker.pose.orientation.w = pose.orientation.w
    return marker

def moveTo(angles):
    left_arm = baxter_interface.Limb("left")
    left_arm.move_to_joint_positions(angles)

#These are recorded positions for the arm that have 3D data for the QR code
#They probably have to be updated if the camera position changes. 
lower_left = {"left_s0" : -0.723271940662,
               "left_s1" : 0.353582571204,
               "left_e0" : 1.43925747259,
               "left_e1" : 1.7809516927,
               "left_w0" : -2.02063619053,
               "left_w1" : 1.37367979398,
               "left_w2" : -2.03252454163}

upper_left  = {"left_s0" : -0.501228221869,
               "left_s1" : 0.132689338,
               "left_e0" : 1.79974295733,
               "left_e1" : 1.58038370491,
               "left_w0" : -1.99455851717,
               "left_w1" : 1.63407303243,
               "left_w2" : -2.03367502722}

lower_right  = {"left_s0" : 0.0448689379944,
               "left_s1" : 0.0701796209656,
               "left_e0" : 1.24290793196,
               "left_e1" : 1.32420891363,
               "left_w0" : -1.68814585514,
               "left_w1" : 1.25402929266,
               "left_w2" : -3.05914118275}

upper_right  = {"left_s0" : -0.106611664636,
               "left_s1" : -0.187145655908,
               "left_e0" : 1.47722349705,
               "left_e1" : 1.4825924298,
               "left_w0" : -1.50100019923,
               "left_w1" : 1.40742737128,
               "left_w2" : -3.05837419235}


class PoseRegistrator:
    def __init__(self):
        self.qrHypothesis = None
        self.endEffectorHypothesis = None
        self.tf = TransformListener()
        self.errorTrans = None
        self.mut = threading.Lock()
        
    def qrCodeCallback(self, qrPose):
        try:            
            #Wait for a transform from the ruler-measured QR code frame to the detected QR code pose
            #self.tf.waitForTransform("qr_code_frame", qrPose.header.frame_id, rospy.Time(0), rospy.Duration(0))
            tmp = qrPose.header.frame_id
            qrPose.header.frame_id = "qr_code_frame"
            self.tf.waitForTransform("qr_code_frame", qrPose.header.frame_id, rospy.Time(0), rospy.Duration(0))
            
            with self.mut:
                self.errorTrans = self.tf.transformPose(tmp, qrPose)
                self.errorTrans = self.errorTrans.pose
            
        except Exception as e:
            #The failure mode of waitForTransform is to throw a generic Exception. Grrrr. 
            rospy.logwarn("Ignored exception %s", e.message)
            return
    
    def getTransSample(self):
        with self.mut:
            return self.errorTrans

class AveragePose(Pose):
    def __init__(self):
        super(AveragePose, self).__init__()
        self.entries = []
        self.position.x = 0
        self.position.y = 0
        self.position.z = 0
        self.orientation.x = 0
        self.orientation.y = 0
        self.orientation.z = 0
        self.orientation.w = 0
        
    def update(self, new_pose):
        if new_pose is None:
            return
        self.entries.append(new_pose)
        pX = 0 
        pY = 0
        pZ = 0
        oX = 0
        oY = 0
        oZ = 0
        oW = 0
        for entry in self.entries:
            #import pdb; pdb.set_trace()
            pX += entry.position.y
            pY += entry.position[1]
            pZ += entry.position[2]
            oX += entry.orientation[0]
            oY += entry.orientation[1]
            oZ += entry.orientation[2]
            oW += entry.orientation[3]
        self.position.x = pX/len(self.entries)
        self.position.y = pY/len(self.entries)
        self.position.z = pZ/len(self.entries)
        self.orientation.x = oX/len(self.entries)
        self.orientation.y = oY/len(self.entries)
        self.orientation.z = oZ/len(self.entries)
        self.orientation.w = oW/len(self.entries)
    
    def clear(self):
        self.entries = []
        self.position.x = 0
        self.position.y = 0
        self.position.z = 0
        self.orientation.x = 0
        self.orientation.y = 0
        self.orientation.z = 0
        self.orientation.w = 0
        #Update 
if __name__ == '__main__':
    
    rospy.init_node("arm_calibration")

    #Enable the robot
    rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    try:
        rs.enable()
    except Exception, e:
        rospy.logerr(e.strerror)
    # initialize robot hand
    left_hand = baxter_interface.Gripper('left', baxter_interface.CHECK_VERSION)
    # calibrate robot hand
    if not left_hand.calibrated():
        instr = raw_input("Calibrate gripper (Y/n): ")
        if instr == "y" or instr == "Y":
            left_hand.calibrate()
    # open robot hand
    left_hand.open(block=True)
    
    #Set up to get the pose of the left arm later on
    limb = baxter_interface.Limb('left')

    pr = PoseRegistrator()~/qu  
    rospy.Subscriber("/qrcode_pose", PoseStamped, (lambda x: pr.qrCodeCallback(x)))
    
    counts = [0,0,0,0]
    locations = [lower_left, upper_left, lower_right, upper_right]
    averages = [[], [], [], []]
    lastPosition = 0
    random.seed()
    
    while not rospy.is_shutdown():
        #Pick a random location and go there
        lIdx = random.randint(0,3)
        if counts[lIdx] < 3 and lIdx != lastPosition:
            lastPosition = lIdx #Don't visit the same place more than once
            counts[lIdx] += 1 #Keep track of how often the place was visited
            moveTo(locations[lIdx]) #Go to the location
            ap = AveragePose() #Accumulator for the average transform (translation and rotation)
            for ii in range(10): #Get 10 samples
                r = rospy.Duration(0.5) #Each sample is a half second apart
                rospy.sleep(r)
                #ap.update(pr.getTransSample())
                averages[lIdx].append(pr.getTransSample())
        else:
            done = True
            for value in counts:
                if value < 3:
                    done = False
            if done:
                break
            else:
                continue
        
    print "lower_left"
    for entry in averages[0]:
        print entry.position.x,",", entry.position.y,",", entry.position.z,",", entry.orientation.x,",", entry.orientation.y,",", entry.orientation.z,",", entry.orientation.w
    print "upper_left"
    for entry in averages[1]:
        print entry.position.x,",", entry.position.y,",", entry.position.z,",", entry.orientation.x,",", entry.orientation.y,",", entry.orientation.z,",", entry.orientation.w
    print "lower_right"
    for entry in averages[2]:
        print entry.position.x,",", entry.position.y,",", entry.position.z,",", entry.orientation.x,",", entry.orientation.y,",", entry.orientation.z,",", entry.orientation.w
    print "upper_right"
    for entry in averages[3]:
        print entry.position.x,",", entry.position.y,",", entry.position.z,",", entry.orientation.x,",", entry.orientation.y,",", entry.orientation.z,",", entry.orientation.w
        
            
    
    