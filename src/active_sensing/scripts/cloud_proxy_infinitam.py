# Python
from copy import copy
from math import pi
import scipy.io
from scipy.spatial import cKDTree
from numpy import array, concatenate, dot, empty, ones, reshape, zeros
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
# ROS
import tf
import rospy
from std_msgs.msg import Int32 as Int32Msg
# trajopt
import cloudprocpy
# Helping Hands
from infinitam.msg import volume as VolumeMsg
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
# self
from cloud_proxy import CloudProxy

START_ACTIVE_SENSOR = 1
STOP_ACTIVE_SENSOR = 0

class CloudProxyInfinitam(CloudProxy):
  '''A class for interfacing with merged point clouds from the two-camera driver.'''
  
  
  def __init__(self):
    '''Set variables contained in self.'''
    
    CloudProxy.__init__(self)
    
    self.hasVolume = False
    self.volumeMsg = None    
    
    # infinitam parameters: check whether infinitam has lost track
    self.angOffset = 10*(pi/180) # maximum allowed offset between orientations
    self.transOffset = 0.05 # maximum allowed offset between positions

    # create publishers and subscriber to communicate with infinitam
    self.infinitamSub = rospy.Subscriber("/infinitam/volume", VolumeMsg, self.callback)
    self.activeCloudPub = rospy.Publisher("/infinitam/publishVolume", Int32Msg, queue_size=1)    
    
    # Wait for nodes to come online
    print("Waiting for /infinitam/volume to connect ...")
    while self.infinitamSub.get_num_connections() == 0:
      rospy.sleep(1)

  
  def callback(self, msg):
    '''Called by ROS when a point cloud message has arrived on the subscriber.'''        
    if not self.hasVolume:
      self.volumeMsg = msg
      self.hasVolume = True
      print("Received InfiniTAM volume with {} points.".format(self.volumeMsg.nPoints))

  def isGoodTrack(self, fk0, fk1, angOffset, transOffset):
    '''
    Determines if a track is good by comparing the delta infinitam pose to the FK pose.

    @type fk0: 1x7 vector
    @param fk0: the c-space position of the arm at the first viapoint
    @type fk1: 1x7 vector
    @param fk1: the c-space position of the arm at the second viapoint
    @type angOffset: scalar
    @param angOffset: the max allowed angular offset between the two viapoints
    @type transOffset: scalar
    @param transOffset: the max allowed translational offset between the two viapoints 
    @rtype: boolean
    @return: True if the offset is less than the allowed tolerances, False otherwise
    '''
    
    # check for complete track loss
    if numpy.isnan(self.volumeMsg.pose).any():
      print("Complete track loss.")
      return False
    
    # calculate pose difference
    s1Ts0_1 = numpy.dot(numpy.linalg.inv(fk1), fk0)
    s1Ts0_2 = numpy.reshape(self.volumeMsg.pose, (4,4), 'F')
    
    # calculate translational difference
    t1 = s1Ts0_1[0:3,3]; t2 = s1Ts0_2[0:3,3]
    dt = numpy.linalg.norm(t1-t2)
    print("Track uncertainty has translational magnitude of {} meters.".format(dt))
    if dt > transOffset: 
      print " Tracking failed!"
      return False
    
    # calculate rotational difference
    s1Rs0 = numpy.dot(s1Ts0_1[:3,:3], s1Ts0_2[:3,:3].T)
    s1Rs0 = tf.transformations.unit_vector(s1Rs0, axis=1)    
    R = numpy.zeros((4,4))
    R[:3,:3] = s1Rs0; R[3,3] = 1
    S = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_matrix(R))    
    angle, direc, point = tf.transformations.rotation_from_matrix(S)
    dr = abs(angle)
        
    rx1, ry1, rz1 = tf.transformations.euler_from_matrix(s1Ts0_1, 'rxyz')
    rx2, ry2, rz2 = tf.transformations.euler_from_matrix(s1Ts0_2, 'rxyz')
    print("Track uncertainty has rotational magnitude of {} degrees.".format(dr*(180/pi)))
    if dr > angOffset: 
      print("Tracking failed!")
      return False
    
    return True

  def transformVectors(self, vectors, T, isPosition):
    '''Takes a 3xn numpy array and transforms it using T. Set isPosition=False if using normal vectors.'''
    
    nVectors = vectors.shape[1]
    augment = ones((1, nVectors)) if isPosition else zeros((1, nVectors))
    cloud = concatenate((vectors, augment), axis=0)
    cloud = dot(T, cloud)
    return cloud[0:3,:]
  
  def processPoints(self, T, workspace):
    '''
    Takes the InfiniTAM points array and converts it to a matrix of points.
    
    @type T: 4x4 numpy array
    @param T: Base to sensor start pose.
    @type workspace: tuple
    @param workspace: min and max extents of cloud we are interested in in the base frame.
    @return cloud: nx3 numpy array of points.
    '''
   
    # transform points to /base frame
    nVectors = len(self.volumeMsg.points)/3
    cloud = reshape(self.volumeMsg.points, (3, nVectors), 'F')
    cloud = self.transformVectors(cloud, T, True)
    
    # transform normals to /base frame
    normals = reshape(self.volumeMsg.normals, (3, nVectors), 'F')
    normals = self.transformVectors(normals, T, False)
    
    mask = (((((cloud[0,:] >= workspace[0][0]) & (cloud[0,:] <= workspace[0][1])) \
             & (cloud[1,:] >= workspace[1][0])) & (cloud[1,:] <= workspace[1][1])) \
             & (cloud[2,:] >= workspace[2][0])) & (cloud[2,:] <= workspace[2][1])
    cloud = cloud[:,mask].T
    normals = normals[:,mask].T
    
    # apply outlier removal    
    outlierBall=0.025; outlierCount=5
    fullCloud = cloud; cloud = []
    fullNormals = normals; normals = []
    tree = cKDTree(fullCloud)
    indicesList = tree.query_ball_point(fullCloud, outlierBall)
    for i, indicies in enumerate(indicesList):
      if len(indicies) >= outlierCount:
        cloud.append(fullCloud[i])
        normals.append(fullNormals[i])
    return numpy.array(cloud), numpy.array(normals)
  

  def getCloud(self):
    '''Wait for a new cloud and convert it into a numpy nx3 array.'''
    
    if not self.hasVolume:
        return None


    # self.hasCloud = False
    # self.registerSensors()
   
    # print("getCloud waiting for unified cloud...")
    # while not self.hasCloud:
    #   rospy.sleep(0.01)
    
    # cloudTime = self.cloudMsg.header.stamp
    # cloudFrame = self.cloudMsg.header.frame_id
    # cloud = array(list(point_cloud2.read_points(self.cloudMsg)))[:,0:3]
    # mask = numpy.logical_not(numpy.isnan(cloud).any(axis=1))
    # cloud = cloud[mask]
    
    # print("Received unified point cloud with {} points.".format(cloud.shape[0]))
    # self.unregisterSensors()
    return cloud, cloudFrame, cloudTime
  
  def getCloudInBaseFrame(self, node):
    '''Wait for a new cloud and convert it into a numpy nx3 array.'''
    
    # self.hasCloud = False
    # self.registerSensors()
  
    # print("getCloudInBaseFrame waiting for unified cloud...")
    # while not self.hasCloud:
    #   rospy.sleep(0.01)
    
    # #Remove NaN points and transform to base frame
    # cloudFrame = self.cloudMsg.header.frame_id
    # cloud = array(list(point_cloud2.read_points(self.cloudMsg)))[:,0:3]
    # mask = numpy.logical_not(numpy.isnan(cloud).any(axis=1))
    # cloud = cloud[mask]
  
    # #Transform to base frame
    # T = node.lookupTransform("base", cloudFrame)
    # cloud = self.transform(cloud, T)

    # print("Received unified cloud with {} points.".format(cloud.shape[0]))
    # self.unregisterSensors()
    return cloud

  def convertToPointCloud2(self, cloud, normals=None):
    '''
    Transform the Infinitam volume to a point cloud in the /base frame.
    
    @type cloud: 
    @param cloud:   
    '''
    
    header = HeaderMsg()
    header.frame_id = "/base"
    header.stamp = rospy.Time.now()
    
    if normals is None:
      return point_cloud2.create_cloud_xyz32(header, cloud)
    
    # concatenate xyz and normals vertically
    pts = numpy.zeros((cloud.shape[0],6))
    pts[:,0:3] = cloud[:,0:3]
    pts[:,3:6] = normals[:,0:3]
    
    # create message
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('normal_x', 12, PointField.FLOAT32, 1),
              PointField('normal_y', 16, PointField.FLOAT32, 1),
              PointField('normal_z', 20, PointField.FLOAT32, 1)]
    return point_cloud2.create_cloud(header, fields, pts)
  
  def followPartialTrajectory(self, traj, idx1, idx2, mover, viewEffector):
    '''
    Make the robot arm follow a given partial trajectory, while streaming images into Infinitam.
    
    @type traj: list
    @param traj: the list of viapoints
    @type idx1: integer
    @param idx1: index at which to start Infinitam  
    @type idx2: integer
    @param idx2: index at which to stop the arm motion and request the Infinitam volume
    @type mover: Arm
    @param mover: the robot arm
    @type viewEffector: string
    @param viewEffector: the name of the sensor mounted to the robot hand
    '''
    
    fk0 = []
    startSpeed = 0.6; endSpeed = 0.2; speed = startSpeed
    startThresh = 0.20; endThresh = 0.05; thresh = startThresh
    speedStep = (startSpeed - endSpeed)/float(len(traj))
    threshStep = (startThresh - endThresh)/float(len(traj))
    
    for i in xrange(len(traj)):
      print "i:", i, ", speed:", speed, "thresh:", thresh
      
      mover.moveVelocity(traj[i], speed, thresh)
      
      speed -= speedStep
      thresh -= threshStep
      
      # start infinitam
      if i == idx1:
        rospy.sleep(1.0)
        self.activeCloudPub.publish(START_ACTIVE_SENSOR)
        rospy.loginfo("Send START to infinitam")
        self.hasVolume = False
        print "Waiting for infinitam volume ..."
        while not self.hasVolume:
          rospy.sleep(0.01)
        fk0 = self.lookupTransform("base", viewEffector)
        print("Stopped at 1st viapoint.")
      
      # request infinitam volume and check if track lost
      if i == idx2:
        rospy.sleep(1.0)
        self.activeCloudPub.publish(START_ACTIVE_SENSOR)
        rospy.loginfo("Send START to infinitam")
        self.hasVolume = False
        print "Waiting for infinitam volume ..."
        while not self.hasVolume:
          rospy.sleep(0.01)
        fk1 = self.lookupTransform("base", viewEffector)    
        break
    
    return fk0, fk1

  def getAndProcessCloud(self, node, traj, essIdxs, arm, viewEffector):
    '''Gets a point cloud, transforms it into the base frame, and cleans up noise.'''
    #"node" is a grasp planning node
  
    #Move arm along trajectory, starting and stopping infinitam
    fk0, fk1 = self.followPartialTrajectory(traj, essIdxs[1], essIdxs[2], arm, viewEffector)
    
    #At this point, our callback should have been called, so we have a volume to work on

    # if infinitam has lost track, try a different sample pair
    if not self.isGoodTrack(fk0, fk1, self.angOffset, self.transOffset):
        #TODO what do we do here?
        return None

    #Process the volume
    #fk0 from followPartialTrajectory is the transform from the base to the start point
    #fk1 from followPartialTrajectory is the transform from the base to the end point
    activeCloudWorkspace = [(0.00, 1.00), (-0.50, 0.50), (-0.50, 0.50)]
    cloud, normals = self.processPoints(fk1, activeCloudWorkspace)

    pCloud2 = self.convertToPointCloud2(cloud, normals)
    
    return pCloud2
    # cloud = empty((0, 3))
    # for i in xrange(nClouds):
    #   newCloud, newCloudFrame, newCloudTime = self.getCloud()
    #   cloud = concatenate((cloud, newCloud), axis=0)
    
    # cloud = self.filterNearAndFarPoints(cloud, 0.15, 1.00)
    # cloud = self.voxelize(cloud, 0.002)
    # T = node.lookupTransform("base", newCloudFrame, newCloudTime)
    # cloud = self.transform(cloud, T)
    # cloud = self.filterWorkspace(cloud, workspace)
    # #cloud = cloudProxy.filterStatisticalOutliers(cloud, kNeighbors=50, std=1.5)
    # if cloud.shape[0] == 0:
    #   print("Cloud had no points after filtering")
    # else:
    #   print("CLoud is of shape {0}".format(cloud.shape))
    # cloudMsg = self.convertToPointCloud2(cloud)
    # self.cloudVisPub.publish(cloudMsg)        
  
  def registerSensors(self):
    '''Call this first to subscribe to the sensor topic.'''
    
    # self.cloudSub = rospy.Subscriber("/unified_cloud", PointCloud2, self.callback, queue_size=1)
   
    # # Wait for nodes to come online
    # print("Waiting for merged point cloud...")
    # while self.cloudSub.get_num_connections() == 0:
    #   rospy.sleep(0.1)
    pass
  
  def unregisterSensors(self):
    '''Unsubscribe from sensors to avoid interference.'''
    # self.cloudSub.unregister()
    pass