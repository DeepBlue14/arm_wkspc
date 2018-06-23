'''
This module contains functions to select grasps, transform ROS messages, and draw grasps in rviz.
'''

# Python
from copy import copy
from math import pi, cos, floor, sqrt
from operator import attrgetter
from matplotlib import cm
# scipy
import numpy
from numpy import cross
from scipy.linalg import norm
# OpenRAVE
import openravepy
# ROS 
from visualization_msgs.msg import MarkerArray
# self
import plot
from grasp import Grasp


def selectGraspActive(node, arm, graspsNew, graspOffsets, graspsTopK, viewStartConfig):
    
  graspsNew = doubleGrasps(graspsNew)
  print("Doubled grasps, now have {}.".format(len(graspsNew)))
  
  # prune grasps on IK and distance from joint limits
  reachableGrasps, isGraspReachable = filterUnreachableGrasps(graspsNew, graspOffsets[-1], arm, viewStartConfig)
  drawGrasps(graspsNew, isGraspReachable, node.localGraspsPub, [0.0,1.0,0.0])
  print len(reachableGrasps), "are reachable."
  if len(reachableGrasps) == 0:
    return []
  
  # take the top k grasps
  reachableGrasps = sorted(reachableGrasps, key=attrgetter('score'), reverse=True)
  reachableGrasps = reachableGrasps[0:graspsTopK]
    
  print "Selected", len(reachableGrasps), "highest-scoring grasps"
  drawGrasps(reachableGrasps, [True]*len(reachableGrasps), node.bestGraspsPub, [0.0,0.5,0.0])
  
  return reachableGrasps


def selectGraspProbabilistically(grasps, arm, offsets, viewStartConfig, enableVertProb=True):
  '''
  Select the grasp configuration with the highest probability of grasp success.
  
  Grasp success is composed of hand joint limits distance, arm joint limits distance, verticalness, cluster inliers, 
  and grasp position height.
  '''
  
  
  minGraspWidth = 0.025; maxGraspWidth = 0.08
  minGraspWidthDist = 0.015
  minHeight = -0.26; maxHeight = 0.10
  maxDist = 7*(150*(pi/180))
      
  # 1. Calculate grasp success probability.
  graspSuccessProbs = []; graspIdxs = []
      
  for i, grasp in enumerate(grasps):
        
    graspWidthProb = calcSuccessGivenGraspWidth(\
      numpy.array([minGraspWidth, maxGraspWidth]), grasp.width, minGraspWidthDist)
    heightProb = 0.9 + 0.1*((grasp.position[2] - minHeight)/(maxHeight - minHeight))  
    vertProb = max(1 - (1.0/2.0)*abs(grasp.approach[2]+1), 0)
    if not enableVertProb: vertProb = 1
    distProb = max(1 - (1/maxDist)*norm(viewStartConfig-grasp.config), 0)
    graspSuccessProb = graspWidthProb*heightProb*vertProb*distProb    
    print(("Success: {:.2f}, width: {:.2f}, widthProb: {:.2f}, height: {:.2f}, heightProb: {:.2f}, vert: {:.2f}, "\
      + "vertProb: {:.2f}, dist: {:.2f} distProb: {:.2f}.").format(graspSuccessProb, grasp.width, graspWidthProb, \
      grasp.position[2], heightProb, grasp.approach[2], vertProb, norm(viewStartConfig-grasp.config), distProb))
    graspSuccessProbs.append(graspSuccessProb)
    graspIdxs.append(i)
  
  # 2. Sort grasps based on success probability. Iterate sorted grasps and check IK for pregrasp and mid config, and  
  print len(graspIdxs), " grasp configs."
  sortedIndices = numpy.argsort(graspSuccessProbs)
  sortedIndices = numpy.fliplr([sortedIndices])[0] # descending order
  
  selectedGrasp = None
  selectedIdx = -1
  ikOpts = openravepy.IkFilterOptions.CheckEnvCollisions | openravepy.IkFilterOptions.IgnoreEndEffectorCollisions
      
  for i in sortedIndices:
    grasp = grasps[graspIdxs[i]]
        
    midPos = grasp.position - offsets[1]*grasp.approach # mid-pose offset
    midConfigs = arm.calcIKForPQ(midPos, grasp.orientation, ikOpts)    
    if len(midConfigs) == 0:
      print("Has grasp but not mid config. Continuing...")
      continue
    
    preGraspPos = grasp.position - offsets[0]*grasp.approach # pre-grasp offset
    preGraspConfigs = arm.calcIKForPQ(preGraspPos, grasp.orientation, ikOpts)
    if len(preGraspConfigs) == 0:
      print("Has grasp and mid, but not pre-grasp config. Continuing...")
      continue
    
    midConfig = arm.findClosestIK(midConfigs)
    preGraspConfig = arm.findClosestIK(preGraspConfigs)     
    if not isTravelOnLine([preGraspConfig, midConfig, grasp.config], arm):
      print("End effector does not travel in a straight line from . Trying another grasp...")
      continue
        
    selectedGrasp = grasp
    selectedGrasp.midConfig = midConfig
    selectedGrasp.preGraspConfig = preGraspConfig
    selectedIdx = graspIdxs[i]
    print("Selected grasp {0}. Probability of grasp success: {1:.3f}.".format(selectedIdx, graspSuccessProbs[i]))
    break
        
  return selectedIdx, selectedGrasp, graspIdxs


def calcSuccessGivenGraspWidth(limits, width, minLimitsDist):
  minDist = min(abs(width - limits[0]), abs(width - limits[1]))
  maxTerm = max(0.0, minLimitsDist - minDist)
  return 1.0 - maxTerm/minLimitsDist

  
def isTravelOnLine(trajectory, arm, thresh=0.01):
  '''Simulates the controller and checks to see that the end effector lies close to a target line.
  
  - Input trajectory: List of configurations (list of numpy.array) as waypoints to check between.
  - Input mover: A mover object for getting FK solutions.
  - Input thresh: Maximum distance the end effector is allowed to deviate from the line.
  - Returns True if the simulated controller does not take the end effector farther than distance thresh from a straight
    line of travel in workspace. False otherwise.
  '''
  
  thresh = 0.02
  stepSize = 0.01
  
  for i in xrange(len(trajectory)):
    config0 = trajectory[i]
    p0 = arm.calcFK(config0, arm.endEffector)[0:3,3]
    
    if i+1 >= len(trajectory): break
    config1 = trajectory[i+1]
    p1 = arm.calcFK(config1, arm.endEffector)[0:3,3]
    
    errorMagnitude = float('inf')
    config2 = copy(config0)
    
    while errorMagnitude > thresh:
      jointError = config1-config2
      errorMagnitude = norm(jointError)
      config2 += (jointError / errorMagnitude) * stepSize
      p2 = arm.calcFK(config2, arm.endEffector)[0:3,3]
      d = norm(cross(p1-p0,p0-p2)) / norm(p1-p0)
      if d > thresh:
        return False
  
  return True
  
  
def compareGrasps(grasp1, grasp2):
  '''
  Compare the angle between the /base frame's z-axis (vertical) and the grasps' approach vectors.
  
  @type grasp1: agile_grasp/Grasp
  @param grasp1: the first grasp to be compared
  @type grasp2: agile_grasp/Grasp
  @param grasp2: the second grasp to be compared
  @rtype: integer
  @return: 1 if the first grasp approach vector is closer in direction to the z-axis, -1 if it is further, 0 if they 
    are equally close
  '''
  zaxis = numpy.array([0,0,1])  
  approach1 = numpy.array([grasp1.approach.x, grasp1.approach.y, grasp1.approach.z])
  approach2 = numpy.array([grasp2.approach.x, grasp2.approach.y, grasp2.approach.z])
  dot1 = numpy.dot(zaxis, -1.0*approach1)
  dot2 = numpy.dot(zaxis, -1.0*approach2)
  
  if dot1 > dot2:
    return 1
  elif dot1 < dot2:
    return -1
  else:
    return 0


def drawGraspsScored(grasps, pub, duration=100):
    '''
    Draw a list of grasps.
    
    @type grasps: list
    @param grasps: the list of grasps
    @type collides: list
    @param param: indicates for each grasp if it collides or not
    @type pub: ROS publisher
    @param param: the ROS publisher that publishes the grasps  
    '''
    markerArray = MarkerArray()
    red = [1,0,0]
    ns = "graspsLocal"
    for i in xrange(len(grasps)):      
      if grasps[i].score == 0:
        marker = plot.createGraspMarker(ns, i, red, 0.6, grasps[i].position, grasps[i].approach, \
          arrowLength=0.17, scale=0.02, duration=duration)
      else:
        color = numpy.asarray(cm.jet(int(floor(0.19*255))))
        marker = plot.createGraspMarker(ns, i, color, 0.6, grasps[i].position, grasps[i].approach, arrowLength=0.17, \
                                        scale=0.03, duration=duration)
      markerArray.markers.append(marker)
    pub.publish(markerArray)
  

def drawGrasps(grasps, isReachable, pub, rgbReachable, duration=100, notReachableColor = [1,0,0]):
    '''
    Draw a list of grasps.
    
    @type grasps: list
    @param grasps: the list of grasps
    @type collides: list
    @param param: indicates for each grasp if it collides or not
    @type pub: ROS publisher
    @param param: the ROS publisher that publishes the grasps  
    '''
    markerArray = MarkerArray()    
    ns = "graspsLocal"
    for i in xrange(len(grasps)):      
      if isReachable[i]:
        marker = plot.createGraspMarker(ns, i, rgbReachable, 0.6, grasps[i].position, grasps[i].approach, \
          arrowLength=0.15, scale=0.01, duration=duration)
      else:
        marker = plot.createGraspMarker(ns, i, notReachableColor, 0.6, grasps[i].position, grasps[i].approach, \
          arrowLength=0.15, scale=0.01, duration=duration)
      markerArray.markers.append(marker)
    pub.publish(markerArray)


def drawGrasps3Dof(grasps, isReachable, pub, rgbReachable):
    '''
    Draw a list of grasps.
    
    @type grasps: list
    @param grasps: the list of grasps
    @type collides: list
    @param param: indicates for each grasp if it collides or not
    @type pub: ROS publisher
    @param param: the ROS publisher that publishes the grasps  
    '''
    markerArray = MarkerArray()
    ns = "graspsLocal"
    for i in xrange(len(grasps)):
      pose = numpy.zeros((4,4))
      pose[0:3,3] = grasps[i].position
      pose[0:3,0] = grasps[i].axis
      pose[0:3,1] = grasps[i].approach
      pose[0:3,2] = grasps[i].binormal
      markerArray.markers.append(plot.createPoseMarker(ns, i, pose))      
    pub.publish(markerArray)


def drawWaypoints(waypoints, pub):
    '''
    Draw a list of waypoints.
    
    @type grasps: list
    @param grasps: the list of waypoints    
    '''
    markerArray = MarkerArray()
    ns = "waypoints"
    for i in xrange(len(waypoints)):
      pt = [waypoints[i][0,3], waypoints[i][1,3], waypoints[i][2,3]]
      markerArray.markers.append(plot.createSphereMarker(ns, i, [0,0,1], pt))      
    pub.publish(markerArray)

def filterUnreachableGrasps(grasps, offset, mover, nearConfig):
  '''TODO'''
      
  minJointLimitsDist = 3*(pi/180.0)
  jointLimits = mover.joint_limits
  
  reachableGrasps = []
  isGraspReachable = numpy.zeros(len(grasps), 'bool')
  
  nNoIK = 0; nLimits = 0
    
  for i in xrange(len(grasps)):
    pos = grasps[i].position - offset*grasps[i].approach
    configs = mover.calcIKForPQ(pos, grasps[i].orientation, \
      openravepy.IkFilterOptions.CheckEnvCollisions | \
      openravepy.IkFilterOptions.IgnoreEndEffectorCollisions)
    
    minDist = float('inf')
    minConfig = None
    
    if len(configs) == 0:
      nNoIK += 1
    
    for config in configs:
      dist = norm(nearConfig-config) 
      if dist < minDist:
        avoidsJointLimits = True
        for j in xrange(len(config)):
          minDist = min(abs(config[j] - jointLimits[0][j]), abs(config[j] - jointLimits[1][j]))
          if minDist < minJointLimitsDist:
            avoidsJointLimits = False
            nLimits += 1
            break
        if avoidsJointLimits:
          minDist = dist
          minConfig = config
        
    if minConfig is not None:
      grasps[i].config = minConfig
      reachableGrasps.append(grasps[i])
      isGraspReachable[i] = True    
  
  print "# no IK:", nNoIK, "# at limits:", nLimits
  
  return reachableGrasps, isGraspReachable

def findInliers(grasps, axisAlignThresDeg, maxDistThresh, axisAlignThreshDist, minInliers):
  '''TODO: vectorize'''
  
  inliers = numpy.zeros(len(grasps))
  posDeltas = numpy.zeros((len(grasps),3))
  graspsOut = copy(grasps)
    
  for i in xrange(len(grasps)):
    
    grasp = grasps[i]
    numInliers = 0
    posDelta = numpy.zeros(3)
    
    for j in xrange(len(grasps)):
      
      if i == j: continue
      
      g = grasps[j]
      
      # Which hands have an axis within <axisAlignThresDeg> of this one?
      axisAligned = numpy.dot(grasp.axis, g.axis)
      axisAlignedBinary = numpy.abs(axisAligned) > cos(axisAlignThresDeg*pi/180.0)
      
      # Which hands are within <maxDistThres> of this one?
      deltaPos = g.position - grasp.position
      deltaPosMag = sqrt(numpy.sum(numpy.square(deltaPos)))
      deltaPosMagBinary = deltaPosMag <= maxDistThresh
            
      # Which hands are within <axisAlignThresDist> of this one when projected onto the plane orthogonal to this axis?
      axisOrthProj = numpy.identity(3) - numpy.outer(grasp.axis, grasp.axis) 
      deltaPosProj = axisOrthProj * deltaPos
      deltaPosProjMag = sqrt(numpy.sum(numpy.square(deltaPosProj)))
      deltaPosProjMagBinary = deltaPosProjMag <= axisAlignThreshDist 
      
      inlierBinary = (axisAlignedBinary & deltaPosMagBinary) & deltaPosProjMagBinary
      if inlierBinary: 
        numInliers += 1
        posDelta += g.position
      
    if numInliers >= minInliers:
      inliers[i] = numInliers
      posDeltas[i,:] = (posDelta/float(numInliers)) - grasp.position
      graspsOut[i].position = grasp.position + posDeltas[i,:] 
  
  return graspsOut, inliers, posDeltas

  
def doubleGrasps(grasps):
  '''TODO'''
  
  doubledGrasps = []
  for grasp in grasps:    
    grasp2 = Grasp(copy(grasp.position), -grasp.axis, copy(grasp.approach), grasp.width, \
      copy(grasp.offsets), -grasp.binormal)
    doubledGrasps.append(grasp)
    doubledGrasps.append(grasp2)
  return doubledGrasps


def calculateApproachPoints(grasp, arm, endEffector, graspOffsets, ikOpts):
  midPos = grasp.position - graspOffsets[1]*grasp.approach # mid-pose offset
  midConfigs = arm.calcIKForPQ(midPos, grasp.orientation, ikOpts)    
  if len(midConfigs) == 0:
    print("Has grasp but not mid config. Continuing...")
    return None
  
  preGraspPos = grasp.position - graspOffsets[0]*grasp.approach # pre-grasp offset
  preGraspConfigs = arm.calcIKForPQ(preGraspPos, grasp.orientation, ikOpts)
  if len(preGraspConfigs) == 0:
    print("Has grasp and mid, but not pre-grasp config. Continuing...")
    return None
  
  midConfig = arm.findClosestIK(midConfigs)
  preGraspConfig = arm.findClosestIK(preGraspConfigs)     
  if not isTravelOnLine([preGraspConfig, midConfig, grasp.config], arm, endEffector):
    print("End effector does not travel in a straight line from . Trying another grasp...")
    return None
  
  outputGrasp = grasp
  outputGrasp.midConfig = midConfig
  outputGrasp.preGraspConfig = preGraspConfig
  return outputGrasp


def pruneGraspWidth(grasps, minWidth, maxWidth):
  graspsOut = []
  for g in grasps:
    if g.width >= minWidth and g.width <= maxWidth:
      graspsOut.append(g)
  return graspsOut
