'''
This module contains a ROS node that handles communication with the other ROS packages and it contains the <main> 
function.
'''

# Python
import sys
import copy
import math
# scipy
import numpy
import random
from math import pi
# Baxter
import baxter_interface
# ROS
import rospy
# self
import plot
import grasp_selection
from arm import Arm
import grasp as graspModule
from infinitam_cloud import ActiveCloudProxy
from view_planning_node import ViewPlanningNode

# ======================================================================================================================
# ===================================================== MAIN CODE ======================================================
# ======================================================================================================================

# controlling point cloud registration
STOP_CAMERA = 0
START_CAMERA = 1
REGISTER_POINT_CLOUDS = 2

# controlling infinitam
STOP_INFINITAM = 0
START_INFINITAM = 1

# grasp selection parameters
graspsTopK = [20, 30] # view planning, active cloud

# proportional view planning parameters
sensorKeepout = 0.42
nSamples = 200
sphereRelief = 0.02
viewingAngleOrder = 2
maxSampleDist = 7*(30*(pi/180))
maxCSpaceJumpV = 10*(pi/180)
maxCSpaceJumpG = 8*(pi/180)
removePlansBelow = 0.01
minGraspsNeededForCenterGrasp = 10
endEffector = "left_gripper"
viewEffector = "structure_link"

# infinitam parameters: check whether infinitam has lost track
angOffset = 5*(math.pi/180) # maximum allowed offset between orientations
transOffset = 0.05 # maximum allowed offset between positions

# grasping parameterss
graspOffsets = [0.14, 0.10, 0.06]
graspOffsets[1] = graspOffsets[2] + (graspOffsets[0]-graspOffsets[2])/2.0
viewOffset = graspOffsets[2]

# general parameters
isMoving = True # turns arm motion on/off
requiresUserInput = False # turns more user input on/off
useViewer = False # turns the openrave viewer on/off
cameraMode = True # for taking pictures of the robot

# read arguments from terminal
if len(sys.argv) > 1:
  isMoving = sys.argv[1]=="1"
if len(sys.argv) > 2:
  useViewer = sys.argv[2]=="1"

# create high-level objects
print "Initializing ..."
infinitam = ActiveCloudProxy()
node = ViewPlanningNode(isMoving, useViewer, endEffector, "base", graspOffsets, viewOffset, infinitam)
arm = Arm(node.env, node.robot, node.manip, requiresUserInput, "left")
arm.update()

# Show current joint angles
for i in xrange(1000):
  print numpy.array(arm.joint_values)#*(180/pi)
  rospy.sleep(1)
