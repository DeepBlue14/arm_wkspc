from numpy import any, array, isnan, mean, std # scipy

import tf # ros
import rospy # ros
from std_msgs.msg import Int16 as Int16Msg # ros
from geometry_msgs.msg import Point as PointMsg # ros

class LaserDetector:
  '''A class for interfacing with a standard ROS topic point cloud.'''
  
  def __init__(self, node):
    '''TODO'''
    
    self.START_CMD = 1
    self.STOP_CMD = 0
    
    self.hasLaserPoint = True
    
    self.laserToBaseTransform = node.lookupTransform('base', 'rgbd_cam_1_rgb_optical_frame')
    self.laserPointPub = rospy.Publisher("/laser_detector/cmd", Int16Msg, queue_size=1)
    self.laserPointSub = rospy.Subscriber("/laser_detector/point", PointMsg, self.laserPointCallback)     
    
  
  def laserPointCallback(self, msg):
    '''TODO'''
    if not self.hasLaserPoint:            
      self.laserPoint = self.laserToBaseTransform.dot(array([msg.x, msg.y, msg.z, 1.0]))
      self.laserPoint = self.laserPoint[0:3]
      print("Recieved laser point, {}.".format(self.laserPoint))
      self.hasLaserPoint = True
  
  
  def detectStablePoint(self, laserPointWorkspace, laserPointQueueSize, maxLaserPointSigma):
    '''TODO'''
    
    print("Waiting for laser point....")
    
    laserPointQueue = []
    while not rospy.is_shutdown():
      self.hasLaserPoint = False
      self.laserPointPub.publish(Int16Msg(self.START_CMD)) # subscibes to Asus projector
      
      # wait for laser point and request it again every 2s
      waitIdx = 0
      while not self.hasLaserPoint:
        rospy.sleep(0.01); waitIdx += 1
        #if waitIdx % 200 == 0:
        #  self.laserPointPub.publish(Int16Msg(self.START_CMD))
      
      laserPoint = self.laserPoint
      if not any(isnan(laserPoint)) and \
        laserPoint[0] >= laserPointWorkspace[0][0] and laserPoint[0] <= laserPointWorkspace[0][1] and \
        laserPoint[1] >= laserPointWorkspace[1][0] and laserPoint[1] <= laserPointWorkspace[1][1] and \
        laserPoint[2] >= laserPointWorkspace[2][0] and laserPoint[2] <= laserPointWorkspace[2][1]:
          laserPointQueue.append(laserPoint)
          if len(laserPointQueue) > laserPointQueueSize:
            laserPointQueue.pop(0)
      else:
        print("Rejected {} because NaN or outside workspace.".format(laserPoint))
        continue
      
      laserPointStd = max(std(laserPointQueue, axis=0))
      if len(laserPointQueue) < laserPointQueueSize or laserPointStd > maxLaserPointSigma:
        print("Rejected queueSize={}, std={}.".format(len(laserPointQueue), laserPointStd))
        continue
      
      break
    
    self.laserPointPub.publish(self.STOP_CMD) # unsubscribes from Asus projector
    return mean(laserPointQueue, axis=0)
