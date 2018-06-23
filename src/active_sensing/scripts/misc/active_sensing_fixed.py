#!/usr/bin/env python

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
from math import pi
from numpy import array
# Baxter
import baxter_interface
# ROS
import rospy
# self
import plot
from arm import Arm
import grasp as graspModule
from infinitam_cloud import ActiveCloudProxy
from view_planning_node import ViewPlanningNode
from view_planning_antifixed import ViewPlannerProportional

# ======================================================================================================================
# ===================================================== MAIN CODE ======================================================
# ======================================================================================================================

# controlling infinitam
STOP_INFINITAM = 0
START_INFINITAM = 1

# controlling laser point
START_LASER_POINT = 1
laserPointWorkspace = [(0,1.5), (-1,1), (-1,1)]

# grasp selection parameters
graspsTopK = [10, 50] # view planning, active cloud

# proportional view planning parameters

traj = [\
  [1.033, -1.735, 0.516, 1.505, 0.022, 1.761, -0.001],
  [0.959, -1.677, 0.564, 1.505, -0.042, 1.768, 0.093],
  [0.885, -1.620, 0.613, 1.505, -0.107, 1.774, 0.187],
  [0.811, -1.563, 0.661, 1.506, -0.172, 1.781, 0.281],
  [0.737, -1.506, 0.709, 1.506, -0.236, 1.787, 0.376],
  [0.664, -1.448, 0.758, 1.506, -0.301, 1.794, 0.470],
  [0.590, -1.391, 0.806, 1.506, -0.365, 1.800, 0.564],
  [0.516, -1.334, 0.854, 1.506, -0.430, 1.807, 0.658],
  [0.442, -1.277, 0.903, 1.506, -0.495, 1.813, 0.752],
  [0.368, -1.220, 0.951, 1.506, -0.559, 1.820, 0.847],
  [0.295, -1.162, 0.999, 1.506, -0.624, 1.826, 0.941],
  [0.221, -1.105, 1.047, 1.506, -0.688, 1.833, 1.035],
  [0.147, -1.048, 1.096, 1.507, -0.753, 1.839, 1.129],
  [0.073, -0.991, 1.144, 1.507, -0.818, 1.846, 1.223],
  [-0.001, -0.933, 1.192, 1.507, -0.882, 1.852, 1.317],
  [-0.075, -0.876, 1.241, 1.507, -0.947, 1.859, 1.412],
  [-0.148, -0.819, 1.289, 1.507, -1.011, 1.865, 1.506],
  [-0.222, -0.762, 1.337, 1.507, -1.076, 1.872, 1.600],
  [-0.294, -0.745, 1.373, 1.477, -1.073, 1.871, 1.469],
  [-0.366, -0.728, 1.409, 1.448, -1.069, 1.870, 1.339],
  [-0.437, -0.709, 1.445, 1.419, -1.064, 1.868, 1.208],
  [-0.507, -0.689, 1.482, 1.391, -1.057, 1.867, 1.077],
  [-0.577, -0.668, 1.518, 1.363, -1.049, 1.865, 0.946],
  [-0.647, -0.642, 1.552, 1.336, -1.042, 1.861, 0.814],
  [-0.716, -0.611, 1.583, 1.310, -1.033, 1.855, 0.683],
  [-0.785, -0.575, 1.611, 1.286, -1.025, 1.846, 0.552],
  [-0.852, -0.535, 1.636, 1.263, -1.017, 1.836, 0.420],
  [-0.917, -0.490, 1.658, 1.242, -1.008, 1.824, 0.289],
  [-0.980, -0.440, 1.676, 1.224, -1.000, 1.810, 0.158],
  [-1.041, -0.386, 1.691, 1.207, -0.992, 1.794, 0.027],
  [-1.101, -0.330, 1.704, 1.192, -0.983, 1.779, -0.105],
  [-1.160, -0.273, 1.717, 1.178, -0.974, 1.765, -0.236],
  [-1.218, -0.217, 1.730, 1.166, -0.964, 1.752, -0.368],
  [-1.276, -0.160, 1.742, 1.154, -0.953, 1.741, -0.500]]
essIdxs = [0,17,33]

viewOffset = -0.01
maxViewPlanAttempts = 3
maxCSpaceJumpG = 8*(pi/180)
maxCSpaceJumpH = 8*(pi/180)
endEffector = "left_gripper"
viewEffector = "structure_link"

# infinitam parameters: check whether infinitam has lost track
angOffset = 10*(math.pi/180) # maximum allowed offset between orientations
transOffset = 0.05 # maximum allowed offset between positions

# grasping parameters
graspOffsets = [0.14, 0, 0.050]
graspOffsets[1] = graspOffsets[2] + (graspOffsets[0]-graspOffsets[2])/2.0
graspSearchRadius = 0.06

# general parameters
isMoving = True # turns arm motion on/off
requiresUserInput = False # turns more user input on/off
useViewer = False # turns the openrave viewer on/off
cameraMode = True # for taking pictures of the robot

# nominal joint positions to which the arm returns after each attempt
configNominal = [1.03198, -1.73454, 0.51656, 1.50521, 0.02224, 1.75564, -0.00076]
configDrop = [1.22181, -1.825053, 1.693898,  1.178864, 0.30372, 2.083145, -0.182160]

# read arguments from terminal
if len(sys.argv) > 1:
  isMoving = sys.argv[1]=="1"
if len(sys.argv) > 2:
  useViewer = sys.argv[2]=="1"

# create high-level objects
print "Initializing ..."
planner = ViewPlannerProportional()
infinitam = ActiveCloudProxy()
node = ViewPlanningNode(isMoving, useViewer, endEffector, "base", graspOffsets, viewOffset, infinitam)
arm = Arm(node.env, node.robot, node.manip, requiresUserInput, "left")
arm.update()
baxterHand = baxter_interface.Gripper("left", baxter_interface.CHECK_VERSION)

# calibrate robot hand
if not baxterHand.calibrated():
  instr = raw_input("Calibrate robot hand (Y/n): ")
  if instr == "y" or instr == "Y":
    baxterHand.calibrate()
baxterHand.open(block=True)

isRunning = True
numpy.set_printoptions(precision=3)

#Record attempted grasps
attempts = 0
#Did the system go for the right object
correct_obj = 0
#Was the object grasped
successes = 0
#File to record attempts in
#Probably should not just assume this opens the file successfully
import time
fname = "testlog{0}.csv".format(time.strftime("%Y-%m-%d_%H.%M.%S"))
#Write unbuffered, so we don't drop a test
testlog = open(fname, 'w', 0)
testlog.write("time,attempts,correct_obj,successes\n")
    
while isRunning:
  
  # move to nominal joint positions
  if isMoving:
    baxterHand.open()
    arm.moveVelocity(configNominal, 0.6, 0.005)  
    arm.update()
  
  # Planning: ==========================================================================================================
    
  # request laser point
  raw_input('Select your target with the laser and hit Enter: ')
  while True:
    node.hasLaserPoint = False
    node.laserPointPub.publish(START_LASER_POINT)
    print("Waiting for laser point....")
    while not node.hasLaserPoint:
      rospy.sleep(0.01)
    centroid = node.laserPoint
    if not numpy.any(numpy.isnan(centroid)) and \
      centroid[0] >= laserPointWorkspace[0][0] and centroid[0] <= laserPointWorkspace[0][1] and \
      centroid[1] >= laserPointWorkspace[1][0] and centroid[1] <= laserPointWorkspace[1][1] and \
      centroid[2] >= laserPointWorkspace[2][0] and centroid[2] <= laserPointWorkspace[2][1]:
        break

  targHand = graspModule.Grasp(centroid, array([1,0,0]), array([0,0,-1]), 0.45, graspOffsets, graspOffsets[-1])
  baseCloud = array(()); baseCloud=baseCloud.reshape((0,3)) # not using cloud for collision avoidance
  
  sphereMarker = plot.createSphereWithCenterMarker("laserPoint", [1,0,1], targHand.position, graspSearchRadius*2)
  node.sphereMarker.publish(sphereMarker)
  
  # visualize plan
  planner.drawTrajectory(traj, arm, endEffector, [0,0,0.5], [essIdxs[1], essIdxs[2]])
  
  # move arm along trajectory    
  fk0, fk1 = node.followPartialTrajectory(traj, essIdxs[1], essIdxs[2], arm, viewEffector)
  infinitam.activeCloudPub.publish(STOP_INFINITAM)
  print("Send STOP to infinitam.")
  
  # if infinitam has lost track, try a different sample pair
  if not infinitam.isGoodTrack(fk0, fk1, angOffset, transOffset):
    raw_input("Track loss. Press any key to continue.")
    continue
  
  # use infinitam volume to find grasps and move to a grasp pose
  print("Stopped at 2nd viapoint.")
  didMove = node.moveLocally(arm, baxterHand, planner, fk0, maxCSpaceJumpG, maxCSpaceJumpH, endEffector, \
    graspOffsets, configDrop, graspsTopK[1], traj[essIdxs[1]], baseCloud, targHand, graspSearchRadius)
  
  # remove cloud from environment, was added in moveLocally
  arm.removeCloudFromEnvironment()
  
  if not didMove:
    raw_input("Grasp/motion failure. Press any key to continue.")
    continue
    
  # LOGGING: ==========================================================================================================
  attempts += 1
  # query if the grasp was a success
  correct_obj = successes = 0 
  gotValidAnswer = False
  while not gotValidAnswer:
      checkSuccess = raw_input("Did the arm try for the right object? (y/n) ")
      if checkSuccess.lower() == 'y':
          correct_obj = 1
          gotValidAnswer = True
      if checkSuccess.lower() == 'n':
          gotValidAnswer = True
  gotValidAnswer = False
  while not gotValidAnswer:
      checkSuccess = raw_input("Did the arm lift the object? (y/n) ")
      if checkSuccess.lower() == 'y':
          successes = 1
          gotValidAnswer = True
      if checkSuccess.lower() == 'n':
          gotValidAnswer = True
  testlog.write("{0},{1},{2},{3}\n".format(time.strftime("%Y-%m-%d_%H.%M.%S"),attempts,correct_obj,successes))
