'''
This module creates visual markers for rviz.
'''

# system dependencies
import numpy

# ROS dependencies
from geometry_msgs.msg import Point
import rospy
from std_msgs.msg import ColorRGBA
import tf
from visualization_msgs.msg import Marker, MarkerArray


def createMarker(ns, id, rgb, alpha=1.0, frameName="/base", action=Marker.ADD, lifetime=rospy.Duration(60)):
  '''
  Create a visual marker.
  
  @type ns: string
  @param ns: the marker's namspace in rviz
  @type id: integer
  @param id: the marker's ID in rviz
  @type rgb: 1x3 vector
  @param rgb: the marker's color
  @type alpha: the marker's opaquity (1.0 = opaque, 0 = transparent)
  @type frameName: string
  @param frameName: the marker's frame name
  @type action: integer
  @param action: the action that is applied to the marker
  @rtype: rviz visual marker
  @return: the visual marker
  '''
  marker = Marker()
  marker.header.frame_id = frameName
  marker.header.stamp = rospy.get_rostime()
  marker.ns = ns
  marker.id = id  
  marker.action = action
  marker.lifetime = lifetime
  marker.color.r = rgb[0]
  marker.color.g = rgb[1]
  marker.color.b = rgb[2]
  marker.color.a = alpha
  return marker


def createArrowMarker(ns, id, rgb, p, q):
  '''
  Create an arrow marker between two points.
  
  @type ns: string
  @param ns: the marker's namspace in rviz
  @type id: number
  @param id: the marker's ID in rviz
  @type rgb: 1x3 vector
  @param rgb: the marker's color
  @type p: 1x3 vector
  @param p: the point from which the arrow starts
  @type q: 1x3 vector
  @param q: the point at which the arrow ends    
  @rtype: rviz visual marker
  @return: the arrow marker
  '''
  marker = createMarker(ns, id, rgb)
  marker.type = Marker.ARROW
  marker.action = Marker.ADD
  marker.points.append(Point(p[0],p[1],p[2]))
  marker.points.append(Point(q[0],q[1],q[2]))
  marker.scale.x = 0.02
  marker.scale.y = 0.02
  marker.scale.z = 0.02
  return marker
  

def createPointListMarker(ns, id, rgb, pts, indices=[], size=0.03):
  '''
  Create a point-list marker from a list of points.
  
  @type ns: string
  @param ns: the marker's namspace in rviz
  @type id: number
  @param id: the marker's ID in rviz
  @type rgb: 1x3 vector
  @param rgb: the color of the points
  @type pts: list, each element is a 1x3 vector 
  @param pts: the list of points  
  @type size: number
  @param size: the width of the points
  @rtype: rviz visual marker
  @return: the point-list marker
  '''
  marker = createMarker(ns, id, rgb)
  marker.type = Marker.POINTS
  marker.action = Marker.ADD
  marker.pose.orientation.x = 0.0
  marker.pose.orientation.y = 0.0
  marker.pose.orientation.z = 0.0
  marker.pose.orientation.w = 1.0
  marker.scale.x = size
  marker.scale.y = size
  c = 0.1
  step = (1.0-c)/len(pts)
  c = 1.0
  
  for i in xrange(len(pts)):
    marker.points.append(Point(pts[i][0],pts[i][1],pts[i][2]))
    if len(indices) == 0: continue
    
    if i == indices[0]:
      marker.colors.append(ColorRGBA(1, 0, 0, 1))
    elif i == indices[1]:
      marker.colors.append(ColorRGBA(0, 1, 0, 1))
    else:
      marker.colors.append(ColorRGBA(0, 0, c, 1))
    c -= step
    
  return marker


def createConeMarker(ns, id, rgb, finger, cone, planeLength=0.6):
  '''
  Create a cone marker. 
  
  @type ns: string
  @param ns: the marker's namespace in rviz
  @type id: scalar
  @param id: the marker's ID in rviz
  @type rgb: 1x3 vector
  @param rgb: the color of the points
  @type pt: 1x3 vector
  @param pt: a point on the plane
  @type axis: 1x3 vector
  @param axis: the axis that goes along the plane
  @type normal: 1x3 vector
  @param normal: the plane's normal  
  @type length: scalar
  @param length: the length of each side of the rectangle representing the plane
  @rtype: rviz visual marker
  @return: the cone marker
  '''
  coneDirection = [cone.direction[0], cone.direction[1], cone.direction[2], 1]
  rot = tf.transformations.rotation_matrix(cone.angle, finger)
  coneDirection1 = numpy.dot(rot, coneDirection)
  rot = tf.transformations.rotation_matrix(-cone.angle, finger)
  coneDirection2 = numpy.dot(rot, coneDirection)
  
  marker = createMarker(ns, id, rgb)
  marker.type = Marker.LINE_LIST
  marker.action = Marker.ADD
  marker.pose.orientation.x = 0.0
  marker.pose.orientation.y = 0.0
  marker.pose.orientation.z = 0.0
  marker.pose.orientation.w = 1.0
  marker.scale.x = 0.01
  v0 = cone.point + 0.5*planeLength*cone.direction
  v1 = cone.point
  v2 = cone.point + planeLength*coneDirection1[0:3]
  v3 = cone.point + planeLength*coneDirection2[0:3]
  # print "coneDirection1:", coneDirection1
  # print coneDirection2
  marker.points.append(Point(v1[0],v1[1],v1[2]))
  marker.points.append(Point(v0[0],v0[1],v0[2]))
  marker.points.append(Point(v1[0],v1[1],v1[2]))
  marker.points.append(Point(v2[0],v2[1],v2[2]))
  marker.points.append(Point(v1[0],v1[1],v1[2]))
  marker.points.append(Point(v3[0],v3[1],v3[2]))
  return marker
  

def createPlaneMarker(ns, id, rgb, pt, axis, normal, length=0.6):
  '''
  Create a plane marker. 
  
  @type ns: string
  @param ns: the marker's namespace in rviz
  @type id: scalar
  @param id: the marker's ID in rviz
  @type rgb: 1x3 vector
  @param rgb: the color of the points
  @type pt: 1x3 vector
  @param pt: a point on the plane
  @type axis: 1x3 vector
  @param axis: the axis that goes along the plane
  @type normal: 1x3 vector
  @param normal: the plane's normal  
  @type length: scalar
  @param length: the length of each side of the rectangle representing the plane
  @rtype: rviz visual marker
  @return: the plane marker
  '''
  marker = createMarker(ns, id, rgb)
  marker.type = Marker.LINE_STRIP
  marker.action = Marker.ADD
  marker.pose.orientation.x = 0.0
  marker.pose.orientation.y = 0.0
  marker.pose.orientation.z = 0.0
  marker.pose.orientation.w = 1.0
  marker.scale.x = 0.01
  bitangent = numpy.cross(axis, normal)    
  v1 = pt - axis*length - bitangent*length;
  v2 = pt + axis*length - bitangent*length;
  v3 = pt + axis*length + bitangent*length;
  v4 = pt - axis*length + bitangent*length;
  marker.points.append(Point(v1[0],v1[1],v1[2]))
  marker.points.append(Point(v2[0],v2[1],v2[2]))
  marker.points.append(Point(v3[0],v3[1],v3[2]))
  marker.points.append(Point(v4[0],v4[1],v4[2]))
  marker.points.append(Point(v1[0],v1[1],v1[2]))
  return marker


def createSphereMarker(ns, id, rgb, pt, scale=0.03, alpha=1):
  '''
  Create a sphere marker.
  
  @type ns: string
  @param ns: the marker's namespace in rviz
  @type id: scalar
  @param id: the marker's ID in rviz
  @type rgb: 1x3 vector
  @param rgb: the color of the points
  @type pt: 1x3 vector
  @param pt: the centroid of the sphere
  @rtype: rviz visual marker
  @return: the sphere marker       
  '''
  marker = createMarker(ns, id, rgb, alpha)
  marker.type = Marker.SPHERE
  marker.action = Marker.ADD
  marker.pose.position.x = pt[0]
  marker.pose.position.y = pt[1]
  marker.pose.position.z = pt[2]    
  marker.pose.orientation.x = 0.0
  marker.pose.orientation.y = 0.0
  marker.pose.orientation.z = 0.0
  marker.pose.orientation.w = 1.0
  marker.scale.x = scale
  marker.scale.y = scale
  marker.scale.z = scale
  return marker


def createSphereWithCenterMarker(ns, rgb, pt, scale):
  sphere = createSphereMarker(ns, 0, rgb, pt, scale, alpha=0.5)
  center = createSphereMarker(ns, 1, rgb, pt, 0.01, alpha=1)
  markerArray = MarkerArray()
  markerArray.markers.append(sphere)
  markerArray.markers.append(center)
  return markerArray


# def create3DofGraspMarker(ns, id, alpha, grasp, arrowLength=0.15, scale=0.01):
#   ''' TODO '''
#   p = grasp.position  
#   r = p + arrowLength * grasp.axis
#   q = p - arrowLength * grasp.approach
#   s = p + arrowLength * grasp.binormal
#   
#   marker1 = createMarker(ns, id, [1,0,0], alpha)
#   marker1.type = Marker.ARROW
#   marker1.points.append(Point(p[0],p[1],p[2]))
#   marker1.points.append(Point(r[0],r[1],r[2]))
#   
#   marker2 = createMarker(ns, id, [0,1,0], alpha)
#   marker2.type = Marker.ARROW
#   marker2.points.append(Point(p[0],p[1],p[2]))
#   marker2.points.append(Point(q[0],q[1],q[2]))
#   
#   marker3 = createMarker(ns, id, [0,0,1], alpha)
#   marker3.type = Marker.ARROW
#   marker3.points.append(Point(p[0],p[1],p[2]))
#   marker3.points.append(Point(s[0],s[1],s[2]))  
#   
#   markerArray = MarkerArray()
#   markerArray.lifetime = rospy.Duration(100)
#   markerArray.markers.append(marker1)
#   markerArray.markers.append(marker2)
#   markerArray.markers.append(marker3)
#   
#   return markerArray


def createGraspMarker(ns, id, rgb, alpha, center, approach, arrowLength=0.15, scale=0.01, duration=100):
  '''
  Create a grasp marker.
  
  @type ns: string
  @param ns: the marker's namespace in rviz
  @type id: scalar
  @param id: the marker's ID in rviz
  @type rgb: 1x3 vector
  @param rgb: the color of the points
  @type center: 1x3 vector
  @param center: the grasp position 
  @type approach: 1x3 vector
  @param approach: the grasp approach vector 
  '''
  p = center
  q = p - arrowLength*approach
  marker = createMarker(ns, id, rgb, alpha)
  marker.type = Marker.ARROW
  marker.points.append(Point(p[0],p[1],p[2]))
  marker.points.append(Point(q[0],q[1],q[2]))
  marker.scale.x = marker.scale.y = marker.scale.z = scale
  marker.lifetime = rospy.Duration(duration)
  return marker


def createDeleteMarker(ns, id):
  '''
  Create a transparent marker.
  
  @type ns: string
  @param ns: the marker's namespace in rviz
  @type id: scalar
  @param id: the marker's ID in rviz
  @rtype: rviz visual marker
  @return: the marker with alpha=0 (transparent)       
  '''
  marker = createMarker(ns, id, [0,0,0], 0)
  marker.scale.x = marker.scale.y = marker.scale.z = 0.01 
  return marker


def createPoseMarker(ns, id, pose, lineLength=0.10, lineWidth=0.01, alpha=1.0):
  '''
  Create a three-perpendicular-lines-marker for a given pose.
  
  The axes of the pose are drawn in the following colors: x-axis: red, y-axis: green, z-axis: blue.
  
  @type ns: string
  @param ns: the marker's namespace in rviz
  @type id: scalar
  @param id: the marker's ID in rviz
  @type pose: 4x4 matrix
  @param pose: the given pose
  @rtype: rviz visual marker
  @return: the marker with alpha=0 (transparent)       
  '''
  marker = createMarker(ns, id, [0,0,0])
  marker.type = Marker.LINE_LIST
  marker.scale.x = lineWidth # width of line segment
  origin = pose[0:3,3]
  x = origin + lineLength*pose[0:3,0]
  y = origin + lineLength*pose[0:3,1]
  z = origin + lineLength*pose[0:3,2]
  marker.points.append(Point(origin[0],origin[1],origin[2]))
  marker.points.append(Point(x[0],x[1],x[2]))
  marker.points.append(Point(origin[0],origin[1],origin[2]))
  marker.points.append(Point(y[0],y[1],y[2]))
  marker.points.append(Point(origin[0],origin[1],origin[2]))
  marker.points.append(Point(z[0],z[1],z[2]))
  marker.colors.append(ColorRGBA(1, 0, 0, alpha))
  marker.colors.append(ColorRGBA(1, 0, 0, alpha))
  marker.colors.append(ColorRGBA(0, 1, 0, alpha))
  marker.colors.append(ColorRGBA(0, 1, 0, alpha))
  marker.colors.append(ColorRGBA(0, 0, 1, alpha))
  marker.colors.append(ColorRGBA(0, 0, 1, alpha))
  return marker
