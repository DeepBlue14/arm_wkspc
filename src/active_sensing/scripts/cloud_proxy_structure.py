# scipy
import numpy
from numpy import array, concatenate, empty, zeros
# ROS
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
# self
from cloud_proxy import CloudProxy

class CloudProxyStructure(CloudProxy):
  '''A class for interfacing with a standard ROS topic point cloud.'''
  
  
  def __init__(self):
    '''Set variables contained in self.'''
    
    CloudProxy.__init__(self)
    
    self.hasCloud = True
    self.cloudSub = None
    self.cloudMsg = None    
    
    self.registerSensors()
    self.cloudVisPub = rospy.Publisher("/cloud_rviz", PointCloud2, queue_size=1)
  
  
  def callback(self, msg):
    '''Called by ROS when a point cloud message has arrived on the subscriber.'''
    
    if not self.hasCloud:
      self.cloudMsg = msg
      self.hasCloud = True
  
  
  def getCloud(self, ):
    '''Wait for a new cloud and convert it into a numpy nx3 array.'''
    
    self.hasCloud = False
    while not self.hasCloud:
      rospy.sleep(0.01)
    
    cloudTime = self.cloudMsg.header.stamp
    cloudFrame = self.cloudMsg.header.frame_id
    cloud = array(list(point_cloud2.read_points(self.cloudMsg)))[:,0:3]
    mask = numpy.logical_not(numpy.isnan(cloud).any(axis=1))
    cloud = cloud[mask]
    
    print("Received Structure cloud with {} points.".format(cloud.shape[0]))
    return cloud, cloudFrame, cloudTime
  
  
  def getAndProcessCloud(self, node, nClouds, workspace):
    '''Gets a point cloud, transforms it into the base frame, and cleans up noise.'''
    
    rospy.sleep(10.0) # give arm time to settle    
    
    cloud = empty((0, 3))
    for i in xrange(nClouds):
      newCloud, newCloudFrame, newCloudTime = self.getCloud()
      cloud = concatenate((cloud, newCloud), axis=0)
    
    cloud = self.filterNearAndFarPoints(cloud, 0.15, 1.00)
    cloud = self.voxelize(cloud, 0.002)
    T = node.lookupTransform("base", newCloudFrame, newCloudTime)
    cloud = self.transform(cloud, T)
    cloud = self.filterWorkspace(cloud, workspace)
    #cloud = cloudProxy.filterStatisticalOutliers(cloud, kNeighbors=50, std=1.5)
    if cloud.shape[0] == 0:
      print("Cloud had no points after filtering")
    else:
      print("CLoud is of shape {0}".format(cloud.shape))
    cloudMsg = self.convertToPointCloud2(cloud)
    self.cloudVisPub.publish(cloudMsg)    
    
    return cloud
    
  
  def registerSensors(self):
    '''Call this first to subscribe to the sensor topic.'''
    
    self.cloudSub = rospy.Subscriber("/structure/depth/points", PointCloud2, self.callback, queue_size=1)
    
    # Wait for nodes to come online
    print("Waiting for structure to connect ...")
    while self.cloudSub.get_num_connections() == 0:
      rospy.sleep(0.1)
  
  
  def unregisterSensors(self):
    '''Unsubscribe from sensors to avoid interference.'''
    
    self.cloudSub.unregister()