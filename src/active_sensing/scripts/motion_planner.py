'''
This module is a generic motion planner that uses trajopt.
'''

from copy import copy, deepcopy # python
from math import acos, sqrt # python

import numpy # scipy
from numpy.linalg import norm # scipy
from numpy import array, cross, diff, dot, exp, eye, linspace, pi, unique # scipy
from matplotlib import pyplot # matplotlib
from mpl_toolkits.mplot3d import Axes3D # matplotlib

import rospy # ros
import tf # ros
from visualization_msgs.msg import Marker # ros

import json # trajopt
import trajoptpy # trajopt
import openravepy # openrave

import plot # self


class Hand:
  '''
  A robot hand configuration represented by the hand center, the finger directions and the finger normals.
  '''
  
  def __init__(self, grasp, offset):
    '''
    Initialize the hand configuration based on a grasp message.
    
    @type grasp: 
    @param grasp: 
    '''
        
    self.finger = numpy.array([grasp.approach.x, grasp.approach.y, grasp.approach.z])
    
    if 'pose' in dir(grasp):
      self.centerOriginal = numpy.array([grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z])
    else:
      self.centerOriginal = numpy.array([grasp.top.x, grasp.top.y, grasp.top.z])
    
    self.viewCenter = self.centerOriginal - (offset-0.01)*self.finger
    #self.center = self.centerOriginal - offsets[1]*self.finger
    normal = numpy.cross(numpy.array([grasp.axis.x, grasp.axis.y, grasp.axis.z]), self.finger)
    self.fingerNormals = (normal, -normal)
    
    # colums are ordered according to /right_gripper frame
    rot = numpy.eye(4)
    rot[0:3,0] = [grasp.axis.x, grasp.axis.y, grasp.axis.z]    
    rot[0:3,2] = [grasp.approach.x, grasp.approach.y, grasp.approach.z]
    rot[0:3,1] = numpy.cross(rot[0:3,2], rot[0:3,0])    
    quat = tf.transformations.quaternion_from_matrix(rot)
    quat /= numpy.linalg.norm(quat)        
    self.orientation = numpy.array([quat[3], quat[0], quat[1], quat[2]])


class PlanData:
  '''
  Information about a plan and planning algorithm execution.
  '''
  
  def __init__(self, hasConverged, trajCost, trajInCollision, trajTime, samplingTime, totalTime):
    '''
    Constructor for PlanData.
    
    @type hasConverged: bool
    @param hasConverged: True if TrajOpt converged, False otherwise
    @type trajCost: real number 
    @param trajCost: the length of the trajectory in c-space in radians
    @type trajInCollision: bool
    @param trajInCollision: True if the trajectory is known to contact an object in the environment, False otherwise
    @type trajTime: real number
    @param trajTime: execution time in seconds for running trajectory optimziation
    @type samplingTime: real number
    @param samplingTime: execution time in seconds for running hand selection and view point sampling
    @type totalTime: real number
    @param totalTime: the total time in seconds for running the planning algorithm
    '''    
    self.hasConverged = hasConverged
    self.trajCost = trajCost
    self.trajInCollision = trajInCollision
    self.trajTime = trajTime
    self.samplingTime = samplingTime
    self.totalTime = totalTime


class MotionPlanner:
  '''A class for basic motion planning routines.'''
  
    
  def __init__(self, maxCSpaceJump, timeout):
    '''Constructor for view planner.'''
    
    self.maxCSpaceJump = maxCSpaceJump
    self.timeout = timeout
    self.trajectoryPub = rospy.Publisher("/trajectory", Marker, queue_size=1)
    self.samplesPub = rospy.Publisher("/planning_samples", Marker, queue_size=1)
  
  
  def planRrt(self, configStart, configEnd, mover):
    '''TODO'''
    
    print("Planning with RRT* ...")    
    
    planner = openravepy.RaveCreatePlanner(mover.env, 'OMPL_RRTstar')

    # Setup the planning instance.
    params = openravepy.Planner.PlannerParameters()

    params.SetRobotActiveJoints(mover.robot)
    params.SetInitialConfig(configStart)
    params.SetGoalConfig(configEnd)

    # Set the timeout and planner-specific parameters. You can view a list of
    # supported parameters by calling: planner.SendCommand('GetParameters')
    params.SetExtraParameters('<range>' + str(self.maxCSpaceJump) + '</range>')
    params.SetExtraParameters('<time_limit>' + str(self.timeout) + '</time_limit>')

    isGoalValid = planner.InitPlan(mover.robot, params)
    if not isGoalValid:
      print("planRrt: End configuration is invalid.")
      return None
    
    # invoke the planner
    traj = openravepy.RaveCreateTrajectory(mover.env, '')
    result = planner.PlanPath(traj)
    if not result == openravepy.PlannerStatus.HasSolution:
      print("planRrt: No solution found.")
      return None
    
    traj_waypoints = []
    for i in xrange(traj.GetNumWaypoints()):
        traj_waypoints.append(traj.GetWaypoint(i)[:7])
    subsampled_traj = traj_waypoints
    subsampled_traj = self.subsampleTraj(traj_waypoints)

    return subsampled_traj
  
  
  def planTrajopt(self, configStart, configEnd, arm):
    '''TODO'''
    
    print("Planning with trajopt ...")     
    
    trajLen = int((1.0/self.maxCSpaceJump)*norm(configStart-configEnd))+2
    
    request = {
      "basic_info" :
        {"n_steps" : trajLen, "manip" : arm.name+"_arm", "start_fixed" : True},
      "costs" : [
        {"type" : "joint_vel", "params" : {"coeffs" : [4]}},
        {"type" : "collision", "name" : "cont_coll", "params" :
          {"continuous" : True, "coeffs" : [4], "dist_pen" : [0.02]}}
        ],
      "constraints" : [
      {"type" : "joint", "params" : {"vals" : configEnd.tolist() }
      }
      ],
      "init_info" : {"type" : "straight_line", "endpoint" : configEnd.tolist()}
    }
    
    arm.robot.SetDOFValues(configStart, arm.manip.GetArmIndices())
    s = json.dumps(request) # convert dictionary into json-formatted string
    prob = trajoptpy.ConstructProblem(s, arm.env)
    result = trajoptpy.OptimizeProblem(prob)
    trajectory = result.GetTraj()
    
    return trajectory
  
  
  def drawTrajectory(self, traj, mover, endEffector, rgb, indices=[-1,-1]):
    '''TODO'''
    
    pts = []
    for q in traj:
      pts.append(mover.calcFK(q, endEffector)[0:4,3])
    marker = plot.createPointListMarker("trajectory", 0, [0,0,1], pts, indices)
    self.trajectoryPub.publish(marker)
  
  
  def drawSamples(self, samples):    
    self.samplesPub.publish(plot.createPointListMarker("samples", 0, [1.0,1.0,1.0], samples, size=0.01))
  
  
  def hierarchicalPlanAndMove(self, targetConfig, arm, ignoreSoftCollision=False):
    '''TODO'''
    
    currentConfig = array(arm.joint_values)
    if arm.isInCollision(currentConfig):
      print("The current position of the arm is believed to be in collision.")
    if not arm.isInJointLimits(currentConfig):
      print("The current position of the arm is not within the joint limits.")    
    
    # linear plan
    traj = self.linearTrajectory(currentConfig, targetConfig)
    inCollision = IsPathInCollision(traj, arm, ignoreSoftCollision)
    method_type = "linear"
    
    # trajopt plan
    if inCollision:
      print("Linear plan in collision, using Trajopt.")
      traj = self.planTrajopt(currentConfig, targetConfig, arm)
      inCollision = IsPathInCollision(traj, arm, ignoreSoftCollision)
      method_type = "trajopt"
    
    # rrt plan
    ''''if inCollision:
      traj = self.planRrt(currentConfig, targetConfig, arm)
      inCollision = True if traj is None else IsPathInCollision(traj, arm, ignoreSoftCollision)
      method_type = "RRT"'''
    
    if inCollision:
      print("Trajopt plan also in collision, hierarchicalPlanAndMove aborted.")
      return False
    
    # subsample trajectory to increase number of viapoints
    self.drawTrajectory(traj, arm, arm.endEffector, [0,0,0.5])
    
    print "Using", method_type, "planner."
    arm.followTrajectory(traj)
    print "Done following trajectory"
    return True
  
  
  def linearTrajectory(self, configStart, configEnd):
    '''TODO'''
    
    trajLen = int((1.0/self.maxCSpaceJump)*norm(configStart-configEnd))+2
    
    steps = linspace(0, 1, trajLen); traj = []
    for i in xrange(trajLen):
      traj.append((1-steps[i])*configStart + steps[i]*configEnd)
    
    return traj
  
  
  def subsampleTraj(self, traj):
    '''TODO'''
    
    new_traj = []
    for i in xrange(len(traj)-1):
      subTraj = self.linearTrajectory(traj[i], traj[i+1])
      new_traj += subTraj
    return new_traj

# UTILITIES=============================================================================================================


def ClosestDistance(configs1, configs2):
  '''Given two lists of configurations, finds the distance between the closest pair.
  
  - Input configs1: List of configurations.
  - Input configs2: List of configurations.
  - Returns dBest: Shortest L2 distrance between configs1 and configs2.
  '''
  
  dBest = float('inf')
  for c1 in configs1:
    for c2 in configs2:
      v = c1-c2
      d = dot(v,v)
      if d < dBest: dBest = d
  return sqrt(dBest)


def ClosestPair(configs1, configs2):
  '''Finds a pair of configurations that are closest.
  
  - Input configs1: List of candidates for the first item in the pair. (List of numpy arrays.)
  - Input configs2 List of candidates for the second item in the pair. (List of numpy arrays.)
  - Returns closestPair: Two-element tuple of the closest configurations (L2 in c-space).
  '''
  
  dBest = float('inf'); c1Best = None; c2Best = None
  for c1 in configs1:
    for c2 in configs2:
      v = c1-c2
      d = dot(v,v)
      if d < dBest: c1Best = c1; c2Best = c2; dBest = d
  
  return (c1Best, c2Best)
  

def ComputePathCost(trajectory):
  '''Computes sum of L2 distances between points in a trajectory.
  
  - Input trajectory: List of joint configurations (numpy arrays).
  - Returns pathCost: Sum of L2 distances between points in the trajectory.
  '''
  
  pathCost = 0; config0 = trajectory[0]
  for config1 in trajectory:
    pathCost += norm(config0-config1)
    config0 = config1
  return pathCost


def IsInWorkspaceEllipsoid(A,c,p,tol=0.001):
  '''Determines if a point lies inside of an ellipsoid
  - Input A: 3x3 matrix describing the (not necessarily axis-alinged) ellipsoid.
  - Input c: 3-element vector describing the origin of the ellipsoid.
  - Input p: A point in 3d to check for membership.
  - Input tol: How much error is tolerated in membership check (units?)
  - Returns: True if the point lies within or on the ellipsoid.
  - See: http://www.mathworks.com/matlabcentral/fileexchange/9542-minimum-volume-enclosing-ellipsoid
  - See: https://en.wikipedia.org/wiki/Ellipsoid
  '''
  
  v = p-c
  return dot(dot(v,A),v) <= 1+tol


def GeneratePose(sensorPosition, targetPosition, l=(1,0,0)):
  '''Helps to determine a sensor pose just given a sensor position and view target.
    
  - Input sensorPosition: 3-element desired position of sensor placement.
  - Input targetPosition: 3-element position of object required to view.
  - Input l: Sensor LOS axis in base frame given identity orientation.
  - Returns T: 4x4 numpy array (transformation matrix) representing desired pose of end effector in the base frame.
  '''
  
  v = targetPosition - sensorPosition
  
  vMag = norm(v)
  v = [v[0]/vMag, v[1]/vMag, v[2]/vMag]
  
  k = [l[1]*v[2]-l[2]*v[1], l[2]*v[0]-l[0]*v[2], l[0]*v[1]-l[1]*v[0]]
  theta = acos(l[0]*v[0] + l[1]*v[1] + l[2]*v[2])
  
  q = tf.transformations.quaternion_about_axis(theta, k)
  return openravepy.matrixFromPose(numpy.r_[[q[3],q[0],q[1],q[2]], sensorPosition])
  
  
def GeneratePoseGivenUp(sensorPosition, targetPosition, upAxis):
  '''Generates the sensor pose with the LOS pointing to a target position and the "up" close to a specified up.
    
  - Input sensorPosition: 3-element desired position of sensor placement.
  - Input targetPosition: 3-element position of object required to view.
  - Input upAxis: The direction the sensor up should be close to.
  - Returns T: 4x4 numpy array (transformation matrix) representing desired pose of end effector in the base frame.
  '''
  
  v = targetPosition - sensorPosition
  v = v / norm(v)
  
  u = upAxis - dot(upAxis, v) * v
  u = u / norm(u)
  
  t = cross(u, v)
  
  T = eye(4)
  T[0:3,0] = v
  T[0:3,1] = t
  T[0:3,2] = u
  T[0:3,3] = sensorPosition
  
  return T


def GetCollisionIndices(trajectory, mover, softCollision=False):
  '''Checks collisions between waypoints assuming linear motions in configuration space.
  
  - Input trajectory: List of configurations (list of numpy.array) as waypoints to check between.
  - Input samples: List of Sample objects the plan refers to.
  - Input mover: A mover object with the appropriate environment for checking collisions.
  - Returns collisionIndices: List of points or pairs of points in the trajectory where a collision
    was detected. If a single point, this point is in collision. If a pair of points, the collision
    is between the points. The list is empty if there are no collisions.
  '''
  
  thresh = 0.02
  stepSize = 0.01
  
  if softCollision:
    collisionRoutine = mover.checkCollisionIgnoreHand
    jointsRoutine = lambda config: True
  else:
    collisionRoutine = mover.isInCollision
    jointsRoutine = mover.isInJointLimits
  
  collisionIndices = []
  
  for i in xrange(len(trajectory)):
    config0 = trajectory[i]
    if collisionRoutine(config0) or not jointsRoutine(config0):
      collisionIndices.append(i)
    
    if i+1 >= len(trajectory): break
    config1 = trajectory[i+1]
    
    config2 = copy(config0)
    errorMagnitude = norm(config1-config2)
    
    while errorMagnitude > thresh:
      jointError = config1-config2
      errorMagnitude = norm(jointError)
      config2 += (jointError / errorMagnitude) * stepSize
      if collisionRoutine(config2) or not jointsRoutine(config0):
        collisionIndices.append((i,i+1))
        break
  
  return collisionIndices


def IsPathInCollision(trajectory, mover, ignoreSoftCollision):
  '''TODO'''
  
  indices = GetCollisionIndices(trajectory, mover)
  
  if len(indices) == 0:
    return False
    
  elif ignoreSoftCollision:
    
    print "Collision indices: ", indices    
    
    # get rid of tuples from list
    singleIndices = []
    for i in indices:
      if type(i)==type(()):
        singleIndices.append(i[0])
        singleIndices.append(i[1])
      else:
        singleIndices.append(i)
    
    # if in middle of trajectory, not a soft collision
    if not (0 in singleIndices) and not (len(trajectory)-1) in singleIndices:
      print("Collision in middle of trajectory.")
      return True
      
    # if break in trajectory, not a soft collision
    singleIndices = array(singleIndices)
    if max(diff(singleIndices)) > 1:
      print("Break detected in trajectory.")
      return True
    
    # check collision trajectory ignoring hand
    singleIndices = unique(singleIndices)
    problemTraj = [trajectory[i] for i in singleIndices]
    indices = GetCollisionIndices(problemTraj, mover, softCollision=True)
    print "Soft collision indices: ", indices  
    if len(indices) == 0: return False
  
  return True

def LogicsticSoftThresh(x,center,steepness,reverse):
  '''TODO'''
  
  y = 1.0 / (1.0 + exp(-steepness * (x - center)))
  if reverse: return 1 - y
  return y


def PlotDistVsScore(dists,scores):
  '''TODO'''
  
  ax = pyplot.subplot(111)
  
  sortIndices = numpy.argsort(array(dists))
  
  sortedDists = []; sortedScores = []
  for i in sortIndices:
    sortedDists.append(dists[i])
    sortedScores.append(scores[i])
  
  pyplot.scatter(sortedDists, sortedScores, lw=0)
  
  ax.set_xlabel("w-space L2 norm (m)")
  ax.set_ylabel("viewing angle probability")
  ax.set_title("Distance Vs. Finger Viewing Angle")
  pyplot.show()


def PlotPairScores(scores):
  '''TODO'''
  
  ax = pyplot.subplot(111)
  
  x = numpy.arange(0,len(scores),1)
  y = sorted(scores)
  pyplot.plot(x, y, lw=2)
  
  pyplot.show()


def RayIsClear(camPoint, targPoint, obstacleTree, raySegLen=0.015, reliefFromTarget=0.04):
  '''Returns True if a ray from a camera to a target point is clear in a point cloud.
  
  - Input camPoint: 3-element vector of the array origin.
  - Input targPoint: 3-element vector of the array destination.
  - Input obstacleTree: KDTree object containing the point cloud representing an opaque obstacle.
  - Input raySegLen: Determines how big the ray tube is that must be clear and also the step size.
  - Returns True if the ray is clear and False if a point is in the tube between points.
  '''
  
  ray = camPoint - targPoint
  rayMag = norm(ray)
  unitRay = ray / rayMag
  
  targPoint = targPoint + reliefFromTarget*unitRay
  
  currentLength = raySegLen
  endLength = rayMag - raySegLen
  
  while currentLength < endLength:
    queryPoint = targPoint + unitRay*currentLength
    d = obstacleTree.query(queryPoint)
    # exit early if not visible
    if d[0] < raySegLen: return False
    # exit early if closest point is further than line length
    if d[0] > endLength-currentLength: return True
    currentLength += raySegLen
  
  return True
