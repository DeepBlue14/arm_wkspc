# scipy
import numpy
from numpy import array, concatenate
# ROS
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
# self
from cloud_proxy import CloudProxy

class CloudProxyAsus(CloudProxy):
  '''A class for interfacing with a standard ROS topic point cloud.'''
  
  
  def __init__(self, useTwoSensors):
    '''Set variables contained in self.'''
    
    CloudProxy.__init__(self)
    
    self.useTwoSensors = useTwoSensors
    
    self.cloud1Msg = None
    self.cloud2Msg = None
    self.hasCloud1 = True
    self.hasCloud2 = True
  
  
  def callback1(self, msg):
    '''Called by ROS when a point cloud message has arrived on the subscriber.'''
    
    if not self.hasCloud1:
      self.cloud1Msg = msg
      self.hasCloud1 = True
  
  
  def callback2(self, msg):
    '''Called by ROS when a point cloud message has arrived on the subscriber.'''
    
    if not self.hasCloud2:
      self.cloud2Msg = msg
      self.hasCloud2 = True  
  
  
  def getCloudInBaseFrame(self, node):
    '''Wait for a new cloud and convert it into a numpy nx3 array.'''
    
    # register 1st sensor
    self.cloud1Sub = rospy.Subscriber("/rgbd_cam_1/depth/points", PointCloud2, self.callback1, queue_size=1)
    print("Waiting for Asus-1...")
    rospy.sleep(1.5)
    self.hasCloud1 = False
    while not self.hasCloud1:
      rospy.sleep(0.01)
    
    # unregister sensor to avoid interference
    if self.useTwoSensors:
      self.cloud1Sub.unregister()  

    cloudFrame = self.cloud1Msg.header.frame_id
    cloud = array(list(point_cloud2.read_points(self.cloud1Msg)))[:,0:3]
    mask = numpy.logical_not(numpy.isnan(cloud).any(axis=1))
    cloud = cloud[mask]
    T = node.lookupTransform("base", cloudFrame)
    cloud = self.transform(cloud, T)
    
    if self.useTwoSensors:
      self.cloud2Sub = rospy.Subscriber("/rgbd_cam_2/depth/points", PointCloud2, self.callback2, queue_size=1)
      print("Waiting for Asus-2...")
      rospy.sleep(1.5)
      self.hasCloud2 = False
      while not self.hasCloud2:
        rospy.sleep(0.01)      
      
      self.cloud2Sub.unregister()

      cloudFrame = self.cloud2Msg.header.frame_id
      cloud2 = array(list(point_cloud2.read_points(self.cloud2Msg)))[:,0:3]
      mask = numpy.logical_not(numpy.isnan(cloud2).any(axis=1))
      cloud2 = cloud2[mask]
      T = node.lookupTransform("base", cloudFrame)
      cloud2 = self.transform(cloud2, T)
      cloud = concatenate((cloud, cloud2), axis=0)
    
    print("Received combined Asus cloud with {} points.".format(cloud.shape[0]))
    self.unregisterSensors()
    
    return cloud
  
  
  def registerSensors(self):
    '''Get subscribers to Asus sensors.'''
    
    # start subscribers
    self.cloud1Sub = rospy.Subscriber("/rgbd_cam_1/depth/points", PointCloud2, self.callback1, queue_size=1)
      
    if self.useTwoSensors:
      self.cloud2Sub = rospy.Subscriber("/rgbd_cam_2/depth/points", PointCloud2, self.callback2, queue_size=1)
    
    # wait for nodes to come online
    print("Waiting for Asus-1 to connect ...")
    while self.cloud1Sub.get_num_connections() == 0:
      rospy.sleep(0.1)
    
    if self.useTwoSensors:
      print("Waiting for Asus-2 to connect ...")
      while self.cloud2Sub.get_num_connections() == 0:
        rospy.sleep(0.1)
  
  
  def unregisterSensors(self):
    '''Unsubscribe from sensors to avoid interference.'''
    
    self.cloud1Sub.unregister()
    
    if self.useTwoSensors:
      self.cloud2Sub.unregister()