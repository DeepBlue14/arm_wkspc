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
# ROS
import rospy
# self
import plot
from arm import Arm
from view_planner import ViewPlanner
from laser_detector import LaserDetector
from cloud_proxy_asus import CloudProxyAsus
from grasp_planning_node import GraspPlanningNode
from cloud_proxy_structure import CloudProxyStructure
from cloud_proxy_merged import CloudProxyMerged
from cloud_proxy_infinitam import CloudProxyInfinitam

# ======================================================================================================================
# ===================================================== MAIN CODE ======================================================
# ======================================================================================================================

# controlling laser point
laserPointQueueSize = 10
maxLaserPointSigma = 0.08
laserPointWorkspace = [(0.00, 1.00), (-0.5, 0.5), (-0.5, 0.5)]

# sensor paramters
obstacleCloudWorkspace = laserPointWorkspace
graspCloudWorkspace = laserPointWorkspace
nGraspClouds = 3

# motion parameters
obstacleCubeSize = 0.05
maxCSpaceJump = 8*(pi/180)
planningTimeout = 10;
endEffector = "left_gripper"
viewEffector = "structure_link"

# grasping parameters
graspOffsets = [0.13, 0, 0.050]
graspOffsets[1] = graspOffsets[2] + (graspOffsets[0]-graspOffsets[2])/2.0
graspSearchRadius = 0.06
graspsTopK = 50

# view planning parameters
sensorKeepout = 0.45
nSphereSamples = 500
minViewDownAngle = 0.60 # cos of angle
maxPlanningAttempts = 3
sampleSeparation = 0.22
sampleSeparationTolerance = 0.05

# general parameters
isMoving = True # turns arm motion on/off
useViewer = False # turns the openrave viewer on/off
useTwoAsusSensors = True

# nominal joint positions to which the arm returns after each attempt
configNominal = array([1.03198, -1.73454, 0.51656, 1.50521, 0.02224, 1.75564, -0.00076])
configDrop = array([1.22181, -1.825053, 1.693898,  1.178864, 0.30372, 2.0000, -0.182160])

# read arguments from terminal
if len(sys.argv) > 1:
  isMoving = sys.argv[1]=="1"
if len(sys.argv) > 2:
  useViewer = sys.argv[2]=="1"

# create high-level objects
print "Initializing ..."
rospy.init_node("active_sensing")
arm = Arm("left", endEffector, viewEffector, isMoving, useViewer)
planner = ViewPlanner(maxCSpaceJump, planningTimeout)
node = GraspPlanningNode()
laser = LaserDetector(node)
#asus = CloudProxyAsus(useTwoAsusSensors)
mergedCloud = CloudProxyMerged()
#infinitam = CloudProxyInfinitam()
structure = CloudProxyStructure()

isRunning = True
numpy.set_printoptions(precision=3)

while isRunning:
  
  # move to nominal joint positions
  arm.openGripper()
  planner.hierarchicalPlanAndMove(array(configNominal), arm)
  
  # Target: ========================================================================================
    
  # request laser point
  raw_input("Select the item with the laser and hit Enter...")
  laserPoint = laser.detectStablePoint(laserPointWorkspace, laserPointQueueSize, maxLaserPointSigma)
  
  sphereMarker = plot.createSphereWithCenterMarker("laserPoint", [1,0,1], laserPoint, graspSearchRadius*2)
  node.sphereMarker.publish(sphereMarker)
  
  # request base cloud
  #obstacleCloud = asus.getCloudInBaseFrame(node)
  #node.cloudRvizPub.publish(asus.convertToPointCloud2(obstacleCloud))
  #obstacleCloud = asus.filterWorkspace(obstacleCloud, obstacleCloudWorkspace)
  #if obstacleCloud.shape[0] == 0:
  #  raw_input("Asus cloud has no points! Press any key to continue...")
  
  #Get the obstacle cloud in the base frame, publish it for debugging, and 
  #filter it to only the workspace we want
  obstacleCloud = mergedCloud.getCloudInBaseFrame(node)
  node.cloudRvizPub.publish(mergedCloud.convertToPointCloud2(obstacleCloud))
  obstacleCloud = mergedCloud.filterWorkspace(obstacleCloud, obstacleCloudWorkspace)
  if obstacleCloud.shape[0] == 0:
    raw_input("No points in obstacleCloud after filtering! Press enter to continue...")

  arm.addCloudToEnvironment(obstacleCloud, obstacleCubeSize)
  obstacleCloudTree = cKDTree(obstacleCloud)
  
  # Planning: ======================================================================================
  
  didMove = False
  for planAttemptIdx in xrange(maxPlanningAttempts):
    
    # sample viewpoints
    sample, configs = planner.sampleFirstRandomViewpoint(\
      laserPoint, sensorKeepout, nSphereSamples, minViewDownAngle, obstacleCloudTree, arm)
    
    if sample is None:
      print("No IK solutions after testing {} samples.".format(nSphereSamples))
      continue
    
    # move to viewpoint
    targConfig = arm.findClosestIK(configs)
    didMove = planner.hierarchicalPlanAndMove(targConfig, arm)    
    
    if not didMove:
      print("No motion plan found to move to sample.")
      continue
    
    print("Getting structure cloud")
    # get data and detect grasps
    cloud = structure.getAndProcessCloud(node, nGraspClouds, graspCloudWorkspace)
    grasp = node.detectGrasp(cloud, obstacleCloudTree, sample, targConfig, laserPoint, \
      graspSearchRadius, graspOffsets, graspsTopK, structure, arm)
    
    #Force failure for testing
    #grasp = None

    if grasp is None:
      print("No grasp detected, switching to section of base cloud")

      # Get a subsection of the unified point cloud around the laser point
      laserHalfVolume = 0.05 #In meters
      laserDotSpace = [(laserPoint[0] - laserHalfVolume, laserPoint[0] + laserHalfVolume), 
      (laserPoint[1] - laserHalfVolume, laserPoint[1] + laserHalfVolume), 
      (laserPoint[2] - laserHalfVolume, laserPoint[2] + laserHalfVolume)]
      cloud = mergedCloud.filterWorkspace(obstacleCloud, laserDotSpace)

      # Try to get grasps in that
      grasp = node.detectGrasp(cloud, obstacleCloudTree, sample, targConfig, laserPoint, \
      graspSearchRadius, graspOffsets, graspsTopK, structure, arm)
    
      # Force failure for testing
      # grasp = None

      # if grasp is None: 
      #   print("No grasp in base cloud, switching to Infinitam scan")
      #   #Sample a second point for the scan, starting from the one we're already at
      #   #This was for using infinitam scanning, but getting a section of the base cloud 
      #   #seemed like it was faster to implement
      #   secondPoint, secondPointConfigs = planner.sampleSecondViewpoint(sample, laserPoint, sensorKeepout, nSphereSamples, \
      #      sampleSeparation, sampleSeparationTolerance, minViewDownAngle, obstacleCloudTree, arm)

      #   if secondPoint is None:
      #     print ("No second point found after testing {} samples.".format(nSphereSamples))
      #     continue

      #   #Cargo cult coding! Marcus, what does this do?
      #   targConfig = arm.findClosestIK(secondPointConfigs)
    
      #   #Get a trajectory around the laser point
      #   try:
      #     traj, essIdxs, trajData = planner.planViewTarget(laserPoint, (sample, secondPoint), \
      #       arm.joint_values, sensorKeepout, maxCSpaceJump, arm, viewEffector, False)
      #   except:
      #     print "Exception thrown trying to get trajectory for scan"
      #     continue

      #   # If our plan is in collision, fail
      #   if trajData.inCollision:
      #     print("Trajectory in collision for plan attempt {}.".format(planAttemptIdx))
      #     continue

      #   #Activate infinitam and move to the next point
      #   cloud = infinitam.getAndProcessCloud(node, traj, essIdxs, arm, viewEffector)

      #   #TODO find grasps on that
      #   grasp = node.detectGrasp(cloud, obstacleCloudTree, secondPoint, targConfig, laserPoint, \
      #     graspSearchRadius, graspOffsets, graspsTopK, structure, arm)
    
      #   #Found no grasps in Infinitam scan either, so give up
      #   if grasp is None: 
      #     print("No grasp found using any strategy, giving up")
      #     continue
          
    # execute grasp motions and retreive object
    didMove = node.executeGrasp(grasp, arm, planner)
    
    if not didMove:
      print("Motion failure during grasp execution.")
      continue
    
    didTransport = node.transportObject(grasp, configDrop, arm, planner)
    
    if didMove: break
    raw_input("Grasp/motion failure for attempt {}. Press any key to continue.".format(planAttemptIdx))
  
  # Done: ==========================================================================================
  
  if not didMove:
    print("Failed after {} automatic reattempts.".format(maxPlanningAttempts))
