'''Provides a set of tools for viewing both sides of a hand using trajectory optimization.

References:
- trajopt: http://rll.berkeley.edu/trajopt/doc/sphinx_build/html/

Assembled: Northeastern University, 2015
'''

import time as time_module # Python
from math import pi, exp, acos, sqrt # Python
from copy import copy # Python

import numpy.random # scipy
from scipy.linalg import norm # scipy
from numpy import array, cos, dot, linspace, pi, sin # scipy

import motion_planner
from motion_planner import MotionPlanner # self


import json # trajopt
import trajoptpy # trajopt
import openravepy # trajopt
import trajoptpy.kin_utils as ku #trajopt


# STATE=============================================================================================

class ViewConstraint:
  '''Specifies viewing costs/constraints as input to the trajectory optimization problem.'''
  
  def __init__(self, viewTarget, keepout, mover, viewEffector):
    '''Creates a view constraint.
    
    - Input viewTarget: A point in workspace that the sensor should target (meters).
    - Input keepout: Distance from the viewTarget that the sensor must maintain (meters).
    - Input mover: ArmMover object for computing FK.
    - Input viewEffector: Name of the sensor link in OpenRAVE.
    '''
    
    self.viewTarget = viewTarget
    self.keepout = keepout
    self.mover = mover
    self.viewEffector = viewEffector
  
    self.jointLimitsRightArm = array([
      [-97.499, -123.014, -174.991,  -2.864, -175.267, -89.999, -175.267],
      [ 97.499,   59.988,  174.991, 150.000,  175.267, 119.977,  175.267]])
    self.jointLimitsRightArm[0][5] += 4.0
    self.jointLimitsRightArm[1][5] -= 4.0
    self.jointLimitsRightArm *= pi/180.0
  
  def GetDistanceFromLowerJointLimits(self, configuration):
    '''Returns the distance (in radians) to the lower joint limits.'''
    
    return self.jointLimitsRightArm[0] - configuration
  
  def GetDistanceFromUpperJointLimits(self, configuration):
    '''Returns the distance (in radians) to the upper joint limits.'''
    
    return configuration - self.jointLimitsRightArm[1]
  
  def GetKeepoutPenalty(self, configuration):
    '''Computes workspace distance to the view target and converts to a configuration cost.
    
    - Input configuration: The joint configuration to compute the penalty for.
    - Returns penalty: Cost of this configuration (unitless).
    '''
    
    T = self.mover.calcFK(configuration, self.viewEffector); p = T[0:3,3]
    distance = norm(self.viewTarget - p)
    return self.keepout - distance
  
  def GetOrientationPenalty(self, configuration):
    '''Returns the offset of the end effector x-axis from the target axis.
    
    - Input configuration: The joint configuration to compute the constraint for.
    - Returns error: Difference between currect x-axis and target x-axis.
    '''
    
    T = self.mover.calcFK(configuration, self.viewEffector)
    viewOrient = self.viewTarget - T[0:3,3]
    viewOrient /= norm(viewOrient)
    
    return T[0:3,0] - viewOrient
    
  def PathSatisfiesKeepout(self, trajectory, tol=0.005):
    '''Determines if all points in the trajectory satisfy the distance keepout.
    
    - Input trajectory: List of 7-DOF configurations.
    - Input tol: How much failure is tolerated (meters).
    - Returns: True if all points in the trajectory satisfy the distance keepout, False otherwise.
    '''
    
    for config in trajectory:
      if self.GetKeepoutPenalty(config) > tol:
        return False
    return True
    
  def PathSatisfiesOrientation(self, trajectory, tol=0.005):
    '''Determines if all points in the trajectory are properly oriented towards the view target.
    
    - Input trajectory: List of 7-DOF configurations.
    - Input tol: How much failure is tolerated (unitless [0,1]).
    - Returns: True if the trajectory satisfy the orientation constraint, False otherwise.
    '''
    
    for config in trajectory:
      if (self.GetOrientationPenalty(config) > tol).any():
        return False
    return True

class SamplingData:
  '''Information from running the sampling algoirthm.'''
  
  def __init__(self, time):
    '''Create a SamplingData structure.
    
    - Input time: Time taken for the sampling algorithm to run (seconds).
    '''
    
    self.time = time
    self.value = None # probability that view plan will see both sides of the object

class TrajectoryData:
  '''Information from running the viewpoint trajectory algorithm.'''
  
  def __init__(self, time, cost, inCollision, meetsPosition, meetsKeepout, meetsOrient):
    '''Creates a TrajectoryData structure.
    
    - Input time: Time take for the viewpoint trajectory algorithm to run (seconds).
    - Input cost: Length of the trajectory produced (radians).
    - Input inCollision: True if the returned trajectory is in collision.
    - Input meetsPosition: True if the viewpoints in the trajectory are at the required position.
    - Input meetsKeepout: True if viewing trajectory is outside of the desired keepout.
    - Input meetsOrient: True if the viewing trajectory is oriented toward the hand center.
    '''
    
    self.time = time
    self.cost = cost
    self.inCollision = inCollision
    self.meetsPosition = meetsPosition
    self.meetsKeepout = meetsKeepout
    self.meetsOrient = meetsOrient



class ViewPlanner(MotionPlanner):

  def __init__(self, maxCSpaceJump, timeout):
    '''Constructor for view planner; creates base class instance.'''
    
    MotionPlanner.__init__(self, maxCSpaceJump, timeout)
    
    # possible sensor up choices
    theta = linspace(0, 2*pi, 20); x = cos(theta); y = sin(theta)
    self.upChoices = []
    for i in xrange(len(theta)):
      self.upChoices.append(array([x[i], 0, y[i]]))
  

  
  def sampleFirstRandomViewpoint(self, viewCenter, sensorKeepout, nSamples, minVertDist, cloudTree, arm):
    '''TODO'''  
    
    timeStart = time_module.time()
    
    # 1. Sample points uniformly on the keepout sphere.
      
    sphereSamples = numpy.random.randn(nSamples,3)
    for i in xrange(nSamples):
      sphereSamples[i,:] /= norm(sphereSamples[i,:])
      sphereSamples[i,:] = viewCenter + sensorKeepout * sphereSamples[i,:]
    
    # 2. Randomly permute "up" choices so there is no preference for beginning ones.

    upChoices = numpy.random.permutation(self.upChoices)
    
    # 2. Accept first reachable, collision-free sample.
    
    mySample = None; mySampleConfigs = None
    for i in xrange(nSamples):
      # check if in top of sphere
      los = (sphereSamples[i] - viewCenter) / sensorKeepout
      vertDist = dot(los, array([0,0,1]))
      if vertDist < minVertDist: continue
      # check for occlusions
      if not motion_planner.RayIsClear(\
        sphereSamples[i], viewCenter, cloudTree, raySegLen=0.025, reliefFromTarget=0.10):
          continue
      # check IK
      for up in upChoices:
        T = motion_planner.GeneratePoseGivenUp(sphereSamples[i], viewCenter, up)
        mySampleConfigs = arm.calcIKForTGivenLink(T, arm.viewEffector)
        if len(mySampleConfigs) > 0:
          mySample = sphereSamples[i]
          break
      if mySample is not None: break
    
    time = time_module.time()-timeStart
    print("Completed after checking {}/{} viewpoints in {}s.".format(i+1, nSamples, time))
    
    # return
    
    return mySample, mySampleConfigs

  def sampleSecondViewpoint(self, currentPoint, viewCenter, sensorKeepout, nSamples, sampleSeparation,
    sampleSeparationTolerance, minVertDist, cloudTree, arm):
    
    # for timing how long this function takes
    timeStart = time_module.time()

    # the distance the sample has to be from the first point
    sampleMinDist2 = (sampleSeparation-sampleSeparationTolerance)**2
    sampleMaxDist2 = (sampleSeparation+sampleSeparationTolerance)**2

    #1. Sample 3D points uniformly on the sphere
    sphereSamples = numpy.random.randn(nSamples,3)
    for i in xrange(nSamples):
      sphereSamples[i,:] /= norm(sphereSamples[i,:])
      sphereSamples[i,:] = viewCenter + sensorKeepout * sphereSamples[i,:]

    # 2. Randomly permute "up" choices so there is no preference for beginning ones.

    upChoices = numpy.random.permutation(self.upChoices)
    
    # 3. Accept first reachable, collision-free sample that is far enough away.    
    mySample = None; mySampleConfigs = None
    for i in xrange(nSamples):
      # check if in top of sphere
      los = (sphereSamples[i] - viewCenter) / sensorKeepout
      vertDist = dot(los, array([0,0,1]))
      if vertDist < minVertDist: continue

      # check distance from previously sampled point
      vDiff = sphereSamples[i] - currentPoint
      dist2 = dot(vDiff, vDiff)
      if dist2 > sampleMaxDist2 or dist2 < sampleMinDist2:
        continue

      # check for occlusions
      if not motion_planner.RayIsClear(\
        sphereSamples[i], viewCenter, cloudTree, raySegLen=0.025, reliefFromTarget=0.10):
          continue
      
      # check IK
      for up in upChoices:
        T = motion_planner.GeneratePoseGivenUp(sphereSamples[i], viewCenter, up)
        mySampleConfigs = arm.calcIKForTGivenLink(T, arm.viewEffector)
        if len(mySampleConfigs) > 0:
          mySample = sphereSamples[i]
          break

      if mySample is not None: break

    time = time_module.time()-timeStart
    print("Completed after checking {}/{} second points in {}s.".format(i+1, nSamples, time))
    
    return mySample, mySampleConfigs
   
# OPTIMIZING============================================================================================================

  def planViewTarget(self, viewCenter, samplePair, configStart, keepout, maxCSpaceJump, mover, viewEffector, visualize):
    '''Calls trajectory optimization with viewing constraints for a pair of reachable sample positions.
    
    - Input viewCenter: Point targeted for viewing.
    - Input samplePair: A pair of w-space positions critical to viewing the object.
    - Input configStart: The current configuration of the arm.
    - Input keepout: Distance the sensor should keep from the center of the hand (meters).
    - Input maxCSpaceJump: Estimated c-space distance between waypoints in the trajectory (radians).
    - Input mover: ArmMover object for computing IK and checking collisions.
    - Input viewEffector: Name of the sensor link in OpenRAVE.
    - Input cloud: Point cloud (nx3 array) to add as obstacle. Removes before function returns. Ignores if None.
    - Input visualize: True to run the OpenRAVE visualization for the trajopt run.
    - Returns traj: List of 7-DOF configurations according to OpenRAVE ordering.
    - Returns essIdxs: Indices in the trajectory that correspond to beginning, viewing (x2), and ending points.
    - Returns trajData: Metadata for running this algorithm such as time, cost, and trajectory quality.
    '''
    
    timeStart = time_module.time()
    
    #import pdb; pdb.set_trace()

    # solve ik the two samples samples
    configsPair = []
    for i in xrange(2):
      T = motion_planner.GeneratePose(samplePair[i], viewCenter)
      solutions = ku.ik_for_link(T, mover.manip, viewEffector, return_all_solns=True, filter_options= \
        openravepy.IkFilterOptions.CheckEnvCollisions)
      if len(solutions) == 0:
        raise Exception("No IK solutions found for input sample point.")
      configsPair.append(solutions)
    
    # switch order of pair if 2nd sample is closer
    if motion_planner.ClosestDistance([configStart], configsPair[1]) < \
       motion_planner.ClosestDistance([configStart], configsPair[0]):
      samplePair = (samplePair[1], samplePair[0])
      configsPair = (configsPair[1], configsPair[0])
    
    # create constraints
    constraints = [None, None]
    for i in xrange(2):
      constraints[i] = ViewConstraint(viewCenter, keepout, mover, viewEffector)
    
    # run trajopt
    traj, essIdxs, cost, inCollision, meetsPosition, meetsKeepout, meetsOrient = \
      self.smoothTrajectory(configStart, configsPair, constraints, maxCSpaceJump, mover, viewEffector, visualize)
    
    # return result
    time = time_module.time()-timeStart
    trajData = TrajectoryData(time, cost, inCollision, meetsPosition, meetsKeepout, meetsOrient)
    return traj, essIdxs, trajData

  def smoothTrajectory(self, configStart, configsPair, constraints, maxCSpaceJump, mover, \
    viewEffector, visualize):
    '''Runs trajectory optimization with viewing constraints.
    
    - Input configStart: Initial configuration in trajectory/current configuration of the arm.
    - Input configsPair: A pair of lists of configurations from which to choose the viewing points.
    - Input constraints: A pair of ViewConstraint objects for each viewing point.
    - Input maxCSpaceJump: Estimated c-space distance between waypoints in the trajectory (radians).
    - Input mover: ArmMover object for computing IK and checking collisions.
    - Input viewEffector: Name of the sensor link in OpenRAVE.
    - Input cloud: Point cloud (nx3 array) to add as obstacle. Removes before function returns. Ignores if None.
    - Input visualize: True to run the OpenRAVE visualization for the trajopt run.
    - Returns trajectory: List of 7-DOF configurations according to OpenRAVE ordering.
    - Returns essentialIndices: Indices in the trajectory that correspond to beginning, viewing (x2), and ending points.
    - Returns cost: The L2 length of the trajectory (radians).
    - Returns inCollision: True if the resulting trajectory has a collision, according to controller.
    - Returns meetsPosition: True if the resulting trajectory goes through the viewing points.
    - Returns meetsKeepout: True if the trajectory between viewing points is outside of the keepout.
    - Returns meetsOrient: True if the trajectory between viewing points looks at the target point.
    '''
    
    trajoptpy.SetInteractive(visualize)
    
    if mover.isInCollision(configStart):
      raise Exception("The current position of the arm is believed to be in collision.")
    
    # 1. Deduce waypoint indices, trajectory length, and initialization.
    
    pointsPerRadian = int(round(1.0 / maxCSpaceJump))
    initConfigs = [configStart]; essentialIndices = [0]
    
    pair = motion_planner.ClosestPair(configsPair[0], configsPair[1])    
    selectedConfigs = [configStart, pair[0], pair[1]]
    for i in xrange(1,len(selectedConfigs)):
      nextConfig = selectedConfigs[i]
      prevConfig = initConfigs[-1]
      delta = nextConfig - prevConfig
      nSteps = int(round(norm(delta)*pointsPerRadian))
      essentialIndices.append(essentialIndices[-1]+nSteps+1)
      for j in xrange(nSteps):
        initConfigs.append((prevConfig + (float(j+1)/float(nSteps+1))*delta).tolist())
      initConfigs.append(nextConfig.tolist())
    
    # 2. Specify optimization problem and constraints.
    
    trajLen = essentialIndices[-1]+1
    
    request = {
      "basic_info" :
        {"n_steps" : trajLen, "manip" : mover.name+"_arm", "start_fixed" : True},
      "costs" : [
        {"type" : "joint_vel", "params" : {"coeffs" : [4]}},
        {"type" : "collision", "name" : "cont_coll", "params" :
          {"continuous" : True, "coeffs" : [4], "dist_pen" : [0.02]}}
        ],
      "constraints" : [],
      "init_info" : {"type" : "given_traj", "data" : initConfigs}
    }
    
    # add waypoint constraints
    for i in xrange(2):
      timeStep = essentialIndices[i+1]
      request["constraints"].append({"type" : "joint", "params" : {"vals" : initConfigs[timeStep],
        "timestep" : timeStep }})  
    
    # construct problem
    mover.robot.SetDOFValues(configStart, mover.manip.GetArmIndices())
    initConfigs[0] = mover.robot.GetDOFValues().tolist() # wtf? Errors sometimes without this.
    problem = trajoptpy.ConstructProblem(json.dumps(request), mover.env)
    
    # joint limit constraints
    for i in xrange(1, essentialIndices[2]+1):
      problem.AddConstraint(constraints[0].GetDistanceFromLowerJointLimits,
        [(i,j) for j in xrange(7)], "INEQ", "jointLo_{}".format(i))
      problem.AddConstraint(constraints[0].GetDistanceFromUpperJointLimits,
        [(i,j) for j in xrange(7)], "INEQ", "jointHi_{}".format(i))
    
    # viewing constraints
    for i in xrange(essentialIndices[1]+1, essentialIndices[2]):
      constraint = constraints[0] if i < essentialIndices[2] else constraints[1]
      problem.AddConstraint(constraint.GetKeepoutPenalty,
        [(i, j) for j in xrange(7)], "INEQ", "viewKeep_{}".format(i))
      problem.AddConstraint(constraint.GetOrientationPenalty, 
          [(i,j) for j in xrange(7)], "EQ", "viewOrient_{}".format(i))
    
    # 3. Solve optimization problem and return result.
    
    result = trajoptpy.OptimizeProblem(problem)
    trajectory = result.GetTraj()
    
    # check result
    cost = motion_planner.ComputePathCost(trajectory)
    collision = motion_planner.IsPathInCollision(trajectory, mover, ignoreSoftCollision=True)
    meetsPosition = norm(initConfigs[essentialIndices[1]]-trajectory[essentialIndices[1]]) < 0.005 \
      and norm(initConfigs[essentialIndices[2]]-trajectory[essentialIndices[2]]) < 0.005
    meetsKeepout = constraints[0].PathSatisfiesKeepout(\
      trajectory[essentialIndices[1]+1:essentialIndices[2]])
    meetsOrient = constraints[0].PathSatisfiesOrientation(\
      trajectory[essentialIndices[1]+1:essentialIndices[2]])
    
    if collision:
      print("Detected a collision in the trajectory")
    
    return trajectory, essentialIndices, cost, collision, meetsPosition, meetsKeepout, meetsOrient

  def trajHasClearView(self, traj, targHand, cloud, mover, viewEffector, centerToFinger=0.05):
    '''TODO'''
    
    obstacleTree = cKDTree(cloud)   
    
    for config in traj:
      sensorPoint = mover.calcFK(config, viewEffector)
      sensorPoint = sensorPoint[0:3,3]
      if not RayIsClear(sensorPoint, targHand.viewCenter, obstacleTree):
        return False
    
    return True
