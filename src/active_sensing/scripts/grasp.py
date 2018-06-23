from numpy import array, cross, eye, zeros
from numpy.linalg import norm 
from scipy.io import loadmat
from tf.transformations import quaternion_from_matrix
from agile_grasp2.msg import GraspMsg
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Vector3


class Grasp():
  
  def __init__(self, position, axis, approach, width, offsets, binormal=None):
    '''TODO'''
    
    self.midConfig = None
    self.preGraspConfig = None
    self.score = None
    self.config = None
    self.position = position
    self.approach = approach
    self.axis = axis    
    self.binormal = cross(approach, axis) if binormal is None else binormal
    
    self.width = width
    self.score = 0
    self.offsets = offsets
    self.fingerNormals = (self.binormal, -self.binormal)
    
    # colums are ordered according to /right_gripper frame
    rot = eye(4)
    rot[0:3,0] = axis
    rot[0:3,2] = approach
    rot[0:3,1] = binormal    
    quat = quaternion_from_matrix(rot)
    quat /= norm(quat) 
    self.orientation = array([quat[3], quat[0], quat[1], quat[2]])
  
  
  def __repr__(self):
    return "position: " + str(self.position) + ", score: " + str(self.score) + ", width: " + str(self.width) + ", axis: " + str(self.axis)
  
  
  def toGraspMsg(self):
    '''TODO'''
    
    surface = Point()
    bottom = Point()
    
    top = Point()
    top.x = self.position[0]
    top.y = self.position[1]
    top.z = self.position[2]
    
    axis = Vector3()
    axis.x = self.axis[0]
    axis.y = self.axis[1]
    axis.z = self.axis[2]
    
    approach = Vector3()
    approach.x = self.approach[0]
    approach.y = self.approach[1]
    approach.z = self.approach[2]
    
    binormal = Vector3()
    binormal.x = self.binormal[0]
    binormal.y = self.binormal[1]
    binormal.z = self.binormal[2]
    
    width = Float32(self.width)
    score = Float32(self.score)
    
    msg = GraspMsg()
    msg.surface = surface
    msg.bottom = bottom
    msg.top = top
    msg.axis = axis
    msg.approach = approach
    msg.binormal = binormal
    msg.width = width
    msg.score = score
    
    return msg

def loadFromMatlabFile(fileName, offsets, viewCenterOffset):
  '''TODO'''    
  matData = loadmat(fileName)
  matGrasps = matData["grasps"]
  grasps = []
  posMat = zeros((3,matGrasps.size))
  axisMat = zeros((3,matGrasps.size))    
  for i in xrange(matGrasps.size):
    top = matGrasps["top"][0,i].flatten()
    approach = matGrasps["approach"][0,i].flatten()
    axis = matGrasps["axis"][0,i].flatten()
    binormal = matGrasps["binormal"][0,i].flatten()
    graspWidth = matGrasps["graspWidth"][0,i][0][0]
    score = matGrasps["score"][0,i][0][0]
    grasp = Grasp(top, axis, approach, graspWidth, offsets, viewCenterOffset, binormal)
    grasp.score = score
    grasps.append(grasp)
                
    posMat[:,i] = top
    axisMat[:,i] = axis
    
  return grasps, posMat, axisMat


def loadFromTopic(graspListMsg, offsets):
  '''TODO'''  
  grasps = []
  posMat = zeros((3, len(graspListMsg)))
  axisMat = zeros((3, len(graspListMsg)))
      
  for i in xrange(len(graspListMsg)):
    grasp = Grasp(pointToNumpy(graspListMsg[i].top),
                  pointToNumpy(graspListMsg[i].axis), 
                  pointToNumpy(graspListMsg[i].approach),
                  graspListMsg[i].width.data,
                  offsets, 
                  binormal = pointToNumpy(graspListMsg[i].binormal))
    grasp.score = graspListMsg[i].score.data
    grasps.append(grasp)
              
    posMat[:,i] = grasp.position
    axisMat[:,i] = grasp.axis
    
  return grasps, posMat, axisMat


def pointToNumpy(p):
  return array([p.x, p.y, p.z])
