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
from numpy.linalg import norm
from numpy import array, pi
from scipy.spatial import cKDTree
# Baxter
import baxter_interface
# ROS
import rospy
#For UI
from sensor_msgs.msg import Joy
# self
import plot
from arm import Arm
from infinitam_cloud import ActiveCloudProxy
from view_planning_node import ViewPlanningNode
from view_planning_driving import ViewPlannerDriving



# ======================================================================================================================
# ===================================================== MAIN CODE ======================================================
# ======================================================================================================================

# controlling infinitam
STOP_CAMERA = 0
START_CAMERA = 1

# controlling laser point
laserPointQueueSize = 10
maxLaserPointSigma = 0.10
laserPointWorkspace = [(0.00, 1.00), (-0.50, 0.50), (-0.65, 0.50)]

# motion parameters
obstacleCubeSize = 0.05
maxCSpaceJump = 8*(pi/180)
endEffector = "left_gripper"
viewEffector = "structure_link"

# infinitam parameters: check whether infinitam has lost track
angOffset = 10*(math.pi/180) # maximum allowed offset between orientations
transOffset = 0.05 # maximum allowed offset between positions

# grasping parameters
graspOffsets = [0.13, 0, 0.050]
graspOffsets[1] = graspOffsets[2] + (graspOffsets[0]-graspOffsets[2])/2.0
graspSearchRadius = 0.06
graspsTopK = 50

# view planning parameters
maxPlanningAttempts = 5
sensorKeepout = 0.40
sphereRelief = 0.02
nSphereSamples = 500
sampleSeparation = 0.22
sampleSeparationTolerance = 0.05
maxViewDownAngle = 0.5 # cos of angle

# general parameters
isMoving = True # turns arm motion on/off
useViewer = False # turns the openrave viewer on/off

# nominal joint positions to which the arm returns after each attempt
configNominal = [1.03198, -1.73454, 0.51656, 1.50521, 0.02224, 1.75564, -0.00076]
configDrop = [1.22181, -1.825053, 1.693898,  1.178864, 0.30372, 2.083145, -0.182160]

# Set up joystick UI stuff
# This script only listens for a trigger click, other nodes handle pan/tilt of the laser
joystickClicked = False

def jsCallback(jsData):
  if jsData.buttons[0] == 1:
    joystickClicked = True
  else:
    joystickClicked = False
  
rospy.Subscriber("/joy", Joy, jsCallback)

#Print a message, then wait for either an enter key or joystick button press
def userInput(message):
  print message
  #while not joystickClicked:
    #print joystickClicked
    #Break on enter pressed
    #if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
    #  break
    #rospy.sleep(0.01)
  while True:
    print joystickClicked
    rospy.sleep(0.01)

# read arguments from terminal
if len(sys.argv) > 1:
  isMoving = sys.argv[1]=="1"
if len(sys.argv) > 2:
  useViewer = sys.argv[2]=="1"

# create high-level objects
print "Initializing ..."
planner = ViewPlannerDriving()
infinitam = ActiveCloudProxy()
node = ViewPlanningNode(isMoving, useViewer, endEffector, "base", graspOffsets, infinitam)
arm = Arm(node.env, node.robot, node.manip, "left")
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

while isRunning:
  
  # move to nominal joint positions
  if isMoving:
    baxterHand.open()
    configStart = array(arm.joint_values)
    configEnd = array(configNominal)
    goalDist = norm(configStart-configEnd)
    if goalDist > 2*(pi/180):
      trajLen = int((1.0/maxCSpaceJump)*goalDist) + 1
      traj, cost, inCollision = planner.planSimpleTrajectory(\
        configStart, configEnd, arm, endEffector, trajLen)
      planner.drawTrajectory(traj, arm, endEffector, [0,0,0.5])
      arm.followTrajectory(traj, startSpeed=2.0, endSpeed=0.5, startThresh=0.15, endThresh=0.07)
    arm.update()
  
  # Target: ========================================================================================
    
  # request laser point
  #userInput('Select your target with the laser and hit Enter or click the trigger: ')
  raw_input("Select the item with the laser and hit Enter...")
  centroid = node.detectStableLaserPoint(laserPointWorkspace, laserPointQueueSize, maxLaserPointSigma)
  
  sphereMarker = plot.createSphereWithCenterMarker("laserPoint", [1,0,1], centroid, graspSearchRadius*2)
  node.sphereMarker.publish(sphereMarker)
  
  # request base cloud
  node.hasBaseCloud = False
  node.hasGrasps = False
  node.controlAsus(START_CAMERA)
  print("Waiting for base cloud...")
  while not node.hasBaseCloud:
    rospy.sleep(0.01)  
  node.controlAsus(STOP_CAMERA)
  arm.addCloudToEnvironment(node.baseCloud, obstacleCubeSize)
  baseCloudTree = cKDTree(node.baseCloud)
  
  # Planning: ======================================================================================
  
  didMove = False
  for planAttemptIdx in xrange(maxPlanningAttempts):
	
    # sample a pair
    samplePair, sampleData = planner.sampleViewpoints(centroid, sensorKeepout+sphereRelief, \
      nSphereSamples, sampleSeparation, sampleSeparationTolerance, maxViewDownAngle, arm, \
      viewEffector, baseCloudTree)
    
    if samplePair is None:
      print("No sample pair found.")
      continue
    
    # run trajopt
    traj, essIdxs, trajData = planner.planViewTarget(centroid, samplePair, arm.joint_values, \
      sensorKeepout, maxCSpaceJump, arm, viewEffector, False)
    
    # visualize plan
    print(("sampleTime={}, trajTime={}, trajCost={}, trajInCollision={}, trajMeetsPosition={}, " + \
      "trajMeetsKeepout={}, trajMeetsOrient={}").format(sampleData.time, trajData.time, \
      trajData.cost, trajData.inCollision, trajData.meetsPosition, trajData.meetsKeepout, \
      trajData.meetsOrient))
    
    # visualize plan
    planner.drawTrajectory(traj, arm, endEffector, [0,0,0.5], [essIdxs[1], essIdxs[2]])
    
    # determine plan success
    if trajData.inCollision:
      print("Trajectory in collision for plan attempt {}.".format(planAttemptIdx))
      continue
  
    # Motion: ======================================================================================
    
    # move arm along trajectory
    fk0, fk1 = node.followPartialTrajectory(traj, essIdxs[1], essIdxs[2], arm, viewEffector)
    infinitam.activeCloudPub.publish(STOP_CAMERA)
    print("Send STOP to infinitam.")
    
    # if infinitam has lost track, try a different sample pair
    if not infinitam.isGoodTrack(fk0, fk1, angOffset, transOffset):
      raw_input("Track loss for attempt {}. Press any key to continue.".format(planAttemptIdx))
      continue
    
    # use infinitam volume to find grasps and move to a grasp pose
    print("Stopped at 2nd viapoint.")
    didMove = node.moveLocally(arm, baxterHand, planner, fk0, maxCSpaceJump, endEffector, \
      graspOffsets, configDrop, graspsTopK, traj[essIdxs[1]], baseCloudTree, centroid, graspSearchRadius)
  
    if didMove: break
    raw_input("Grasp/motion failure for attempt {}. Press any key to continue.".format(planAttemptIdx))
  
  # Done: ==========================================================================================
  
  if not didMove:
    print("Failed after {} automatic reattempts.".format(maxPlanningAttempts))
# -*- coding: utf-8 -*-

