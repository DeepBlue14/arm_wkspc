from copy import copy # python
# scipy
import numpy
from scipy.linalg import norm
# trajopt/OpenRave
import openravepy
# ROS
import tf
import rospy
from sensor_msgs.msg import PointCloud2 as PointCloud2Msg
from std_msgs.msg import Int64 as Int64Msg
from visualization_msgs.msg import Marker, MarkerArray
# Helping Hands
from agile_grasp2.msg import GraspListMsg, CloudIndexed
# self
import plot
import grasp_selection
import grasp as graspModule
import motion_planner


class GraspPlanningNode:
  '''ROS node for view planning.'''
  
  
  def __init__(self):
    '''TODO'''
    
    self.hasGrasps = False
    self.grasps = None
    self.infinitamCloud = None
    self.infinitamNormals = None
    self.hasLaserPoint = False
    self.laserPoint = None
    
    self.initRos()
  
  
  def initRos(self):
    '''Initialize ROS node.'''
    
    print("GraspPlanningNode: initializing ROS")
    
    self.baseCloud = None
    self.hasBaseCloud = False

    # listen to grasps from agile_grasp
    self.graspsSubPassive = rospy.Subscriber("/detect_grasps_passive/grasps", GraspListMsg, self.graspsCallback)
    self.graspsSubActive = rospy.Subscriber("/detect_grasps_active/grasps", GraspListMsg, self.graspsCallback)
    self.graspsSubVerified = rospy.Subscriber("/detect_grasps_verified/grasps", GraspListMsg, self.graspsCallback)
       
    # create publisher to publish infinitam volume as point cloud for agile_grasp
    self.cloudIndexedPub = rospy.Publisher("/cloud_infinitam", CloudIndexed, queue_size=1)
    self.cloudRvizPub = rospy.Publisher("/cloud_rviz", PointCloud2Msg, queue_size=1)
    
    # create publishers to draw initial and closest grasp
    self.initialGraspPub = rospy.Publisher("/initial_grasp", Marker, queue_size=1)
    self.closestGraspPub = rospy.Publisher("/closest_grasp", Marker, queue_size=1)
    self.sphereMarker = rospy.Publisher("/search_sphere", MarkerArray, queue_size=1)
    
    # create publishers to draw grasps for local trajectory
    self.localGraspsPub = rospy.Publisher("/grasps_local", MarkerArray, queue_size=1)
    self.bestGraspsPub = rospy.Publisher("/best_grasps", MarkerArray, queue_size=1)
    
    # create publisher to visualize the waypoints that lead to the grasp pose
    self.waypoints_pub = rospy.Publisher("/waypoints", MarkerArray, queue_size=1)
                       
    # create TF listener to receive transforms        
    self.listener = tf.TransformListener()
  
 
  def graspsCallback(self, msg):
    '''TODO'''    
    
    if not self.hasGrasps:
      self.grasps = msg.grasps
      rospy.loginfo("Received %d grasps", len(self.grasps))
      self.hasGrasps = True
  
  
  def lookupTransform(self, fromFrame, toFrame, lookupTime=rospy.Time(0)):
    '''
    Lookup a transform in the TF tree.
    
    @type fromFrame: string
    @param fromFrame: the frame from which the transform is calculated
    @type toFrame: string
    @param toFrame: the frame to which the transform is calculated 
    '''
    
    self.listener.waitForTransform(fromFrame, toFrame, rospy.Time(0), rospy.Duration(10.0))
    (pos,quat) = self.listener.lookupTransform(fromFrame, toFrame, rospy.Time(0))
    tfMat = tf.transformations.quaternion_matrix(quat)
    tfMat[0:3,3] = [pos[0],pos[1],pos[2]]
    return tfMat
  
  
  def detectGrasp(self, cloud, obstacleCloudTree, viewPoint, viewConfig, viewTarget, \
    graspSearchRadius, graspOffsets, graspsTopK, cloudProxy, arm):
    '''TODO'''
    
    # compute cloud normals and region of interest
    normals = cloudProxy.computeNormals(cloud, viewPoint)
    ballIndices = cloudProxy.trimCloudToBallIndices(cloud, viewTarget, graspSearchRadius)
    
    if len(ballIndices) == 0:
      print("No points near laser point.")
      return None
    
    # get grasps from agile
    self.hasGrasps = False
    msg = CloudIndexed()
    cloudMsg = cloudProxy.convertToPointCloud2(cloud, normals) 
    msg.cloud = cloudMsg
    for i in ballIndices:
      msg.indices.append(Int64Msg(i))
    self.cloudIndexedPub.publish(msg)
    
    print("Waiting for grasps from agile_grasp ...")
    while not self.hasGrasps:
      rospy.sleep(0.1)
    
    graspsNew, posMat, axisMat = graspModule.loadFromTopic(self.grasps, graspOffsets)
    
    # use constraints and heuristics to select top grasp
    reachableGrasps = grasp_selection.selectGraspActive(\
      self, arm, graspsNew, graspOffsets, graspsTopK, viewConfig)
    
    # (detect if in middle shelf scenario, do not prefer top grasps)
    preferTopGrasps = True
    upPoint = copy(viewTarget); upPoint[2] = upPoint[2] + 0.34
    preferTopGrasps = motion_planner.RayIsClear( \
      upPoint, viewTarget, obstacleCloudTree, raySegLen=0.015, reliefFromTarget=0.10)
    
    print("It is {} that we prefer top grasps.".format(preferTopGrasps))
    
    # (main grasp selection routine)
    idxTarget, selectedGrasp, isValid = grasp_selection.selectGraspProbabilistically(\
      reachableGrasps, arm, graspOffsets, viewConfig, preferTopGrasps)
    
    if idxTarget < 0:
      print("No pre-grasp and grasp pair is reachable.")
      return None
    
    # visualize selected grasp and return
    fkList = [arm.calcFK(q, arm.endEffector) for q in [\
      selectedGrasp.config, selectedGrasp.midConfig, selectedGrasp.preGraspConfig]]
    grasp_selection.drawWaypoints(fkList, self.waypoints_pub)    
    
    closestGraspMarker = plot.createGraspMarker("closestGrasp", 0, [1,1,1], 1, \
      selectedGrasp.position, selectedGrasp.approach, arrowLength=0.17, scale=0.02)
    self.closestGraspPub.publish(closestGraspMarker)
    
    return selectedGrasp
 
  
  def executeGrasp(self, grasp, arm, motionPlanner):
    '''TODO'''
    if grasp is None:
        print("No grasp given to execute, returning")
        return False
        
    # move to pregrasp
    didMove = motionPlanner.hierarchicalPlanAndMove(grasp.preGraspConfig, arm, ignoreSoftCollision=True)
    if not didMove: return False
    
    # move to grasp more carefully
    traj = [grasp.preGraspConfig, grasp.midConfig, grasp.config]
    arm.followTrajectory(traj, startSpeed=0.08, endSpeed=0.08, startThresh=0.05, endThresh=0.05, \
      forceThresh=425)
    rospy.sleep(0.5)
    
    # close the robot hand
    print("Reached new grasp target. Closing robot hand.")
    arm.closeGripper()
    rospy.sleep(1.0)
    return True
  
  
  def transportObject(self, grasp, dropConfig, arm, motionPlanner):
    '''TODO'''
    
    # move back to pre-grasp config
    traj = [grasp.midConfig, grasp.preGraspConfig]
    arm.followTrajectory(traj, startSpeed=0.15, endSpeed=0.15, forceThresh=400)
    #Essentially regrasping, since executeGrasp already closed the gripper
    arm.closeGripper()

    # Move up a little bit, due largely to the arm seeming to detect being in collision 
    # (possibly with the grasped object), during the move home
    T = arm.calcFK(grasp.preGraspConfig, arm.endEffector)
    T[2,3] = T[2,3] + 0.10
    upConfig = arm.findClosestIK(arm.calcIKForT(T))
    if upConfig is None:
        print("Couldn't find nearest IK for move up")
        return False
    traj = [grasp.preGraspConfig, upConfig]
    arm.followTrajectory(traj, startSpeed=0.15, endSpeed=0.15, forceThresh=400)
    
    print("Moved up a bit, now trying to head home")

    # move home
    didMove = motionPlanner.hierarchicalPlanAndMove(dropConfig, arm, ignoreSoftCollision=True)
    # TODO might only want to move up if we didn't move initially, rather than just 
    # always doing it as above
    if not didMove: return False
    
    # move down
    print("Moving down from drop config.")
    T = arm.calcFK(dropConfig, arm.endEffector)
    T[2,3] = T[2,3] - 0.10
    downConfig = arm.findClosestIK(arm.calcIKForT(T))
    traj = [dropConfig, downConfig]
    arm.followTrajectory(traj, startSpeed=0.20, endSpeed=0.20, startThresh=0.10, endThresh=0.05)
    
    # release object
    #raw_input("Press any key to release object.")
    arm.openGripper()
    rospy.sleep(1)
    
    return True
