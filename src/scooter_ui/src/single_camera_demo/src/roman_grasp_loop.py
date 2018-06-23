#!/usr/bin/python

# Assuming that everything is up and running, this script will wait until it has a 
# grasp suggested by the list of grasps filtered based on the location of the laser 
# in the point cloud, and then will open the gripper, move it to the grasp location,
# grasp the object, and move it to some location. 

import rospy
import tf

import sys
import math
import random
import numpy as np
import pprint as pp
import copy

import baxter_interface
from baxter_core_msgs.msg import JointCommand

from agile_grasp.msg import Grasps

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)

from grasp_selection.srv import *

import tf.transformations

from visualization_msgs.msg import Marker

def createTargetMarker(response, idx):
  grasp = response.grasps.grasps[idx]
  diam = 0.01
  alpha = 1.0
  marker = Marker()
  marker.type = Marker.ARROW
  marker.id = 0
  marker.header.frame_id = "/base"
  marker.header.stamp = rospy.get_rostime()
  marker.lifetime = rospy.Duration.from_sec(60.0)
  marker.action = Marker.ADD
  marker.scale.x = diam  # shaft diameter
  marker.scale.y = diam  # head diameter
  marker.scale.z = 0.01  # head length
  marker.color.r = 1.0
  marker.color.g = 0.0
  marker.color.b = 0.0
  marker.color.a = alpha
  p = Point()
  q = Point()
  p.x = grasp.pose.position.x
  p.y = grasp.pose.position.y
  p.z = grasp.pose.position.z
  q.x = p.x - 0.15 * grasp.approach.x
  q.y = p.y - 0.15 * grasp.approach.y
  q.z = p.z - 0.15 * grasp.approach.z
  marker.points.append(p)
  marker.points.append(q)
  return marker

class WaypointViz:
    def __init__(self):
        #TODO this might only publish one marker at a time
        self.wp_pub = rospy.Publisher('waypoints', Marker, queue_size=1)
        
    def createWaypointMarker(self, waypoint):
        diam = 0.01
        alpha = 1.0
        marker = Marker()
        marker.type = Marker.CYLINDER
        marker.id = waypoint.position.x + waypoint.position.y + waypoint.position.z #Should be unique
        marker.header.frame_id = "/base"
        marker.header.stamp = rospy.get_rostime()
        marker.lifetime = rospy.Duration.from_sec(60.0)
        marker.action = Marker.ADD
        marker.scale.x = diam
        marker.scale.y = diam
        marker.scale.z = diam
        marker.color.r = 0.6
        marker.color.g = 0.29
        marker.color.b = 0.702
        marker.color.a = alpha
        #
        marker.pose = waypoint #does this need a deepcopy?
        return marker
    
    def update(self, waypoint):
        newMarker = self.createWaypointMarker(waypoint)
        self.wp_pub.publish(newMarker)

def moveTo(angles, speed=0.3):
    left_arm = baxter_interface.Limb("left")
    left_arm.set_joint_position_speed(speed)
    left_arm.move_to_joint_positions(angles)

def moveVelocity(self, joint_values_target, speed = 0.3, thresh = 0.1, is_printing=False):
    """
    Velocity controller for Baxter arm movements.
    """
    max_iterations = 5000
    step_size = 2.0
    left_arm = baxter_interface.Limb("left")
    
    for i in range(0, max_iterations):
        #get the current joint angles
        
        joint_values = left_arm.joint_angles()
        
        joint_error = np.array(joint_values)
        joint_velocity_command = joint_error
        
        # calculate joint error, error magnitude, and command velocity
        joint_error = joint_values_target - joint_error
        joint_velocity_command = joint_error * step_size
        error_magnitude = np.linalg.norm(joint_error)
        joint_velocity_command_magnitude = error_magnitude * step_size
        
        # if command velocity magnitude exceeds speed, scale it back to speed
        if joint_velocity_command_magnitude > speed:
          joint_velocity_command = speed * joint_velocity_command / joint_velocity_command_magnitude
        
        # send velocity command to joints
        values_out = [joint_velocity_command[2], joint_velocity_command[3], joint_velocity_command[0], joint_velocity_command[1], joint_velocity_command[4], joint_velocity_command[5], joint_velocity_command[6]]
        values_out = zip(left_arm.joint_names, values_out)
        
        #self.pub.publish(JointCommand(JointCommand.VELOCITY_MODE, values_out, self.joint_names))
        left_arm.set_joint_velocities(values_out)
        
        if is_printing:
          print "iteration:", i, "error_magnitude: ", error_magnitude
        
        # break loop if close enough to target joint values
        if error_magnitude < thresh:
          break
        
        rospy.sleep(0.001)
        
    # need to reset to position mode; o.w. the robot will disable itself
    values_out = [joint_values_target[2], joint_values_target[3], joint_values_target[0], joint_values_target[1],
      joint_values_target[4], joint_values_target[5], joint_values_target[6]]
    self.pub.publish(JointCommand(JointCommand.POSITION_MODE, values_out, self.joint_names))
    
def startPosition():
    # Based on output of recording the arm position, likely in radians
    # time,left_s0,left_s1,left_e0,left_e1,left_w0,left_w1,left_w2,left_gripper
    l_arm_start = {"left_s0" :-1.70463614859,
                   "left_s1" :-1.12249044025,
                   "left_e0" :0.332873830591,
                   "left_e1" :2.17978669709,
                   "left_w0" :3.05799069716,
                   "left_w1" :-1.29583026909,
                   "left_w2" :2.98205864824}
    moveTo(l_arm_start)
    
def midPosition():
    #time,left_s0,left_s1,left_e0,left_e1,left_w0,left_w1,left_w2,left_gripper
    mid_position = {"left_s0" :0.162218468134,
                   "left_s1" :-1.68776235994,
                   "left_e0" :0.589815612268,
                   "left_e1" :2.21698573116,
                   "left_w0" :3.05914118275,
                   "left_w1" :-0.999388482166,
                   "left_w2" :0.743980681274}
    moveTo(mid_position)

def homePosition():
    home_position = {"left_s0" :-0.0579077746765,
                   "left_s1" :-2.14680611019,
                   "left_e0" :-0.0322135965088,
                   "left_e1" :1.93511676171,
                   "left_w0" :-0.00421844716187,
                   "left_w1" :1.76676237043,
                   "left_w2" :-0.0011504855896}
    #This is deliberately throttled to avoid shaking the robot and 
    #breaking the calibration. 
    moveTo(home_position, speed = 0.1)
    
def callBuiltInIK(pos, rot):
    # Try calling the built-in IK
    req = SolvePositionIKRequest()
    req_head = Header(stamp=rospy.Time.now(), frame_id='base')
    req_pose = PoseStamped(header=req_head, pose=Pose(position=pos, orientation=rot))
    req.pose_stamp.append(req_pose)
    
    # Call the services
    rospy.wait_for_service('ExternalTools/left/PositionKinematicsNode/IKService')
    try:
        ik_solver = rospy.ServiceProxy('ExternalTools/left/PositionKinematicsNode/IKService', SolvePositionIK)
        response = ik_solver(req)
        return response
    except rospy.ServiceException, e:
        print "Call failed %s" % e

def grab_object(grasp, visualizer=None):
    #Move to the midpoint 

    # The waypoints allow the arm to approach the object slowly along the chosen approach axis
    num_waypoints = 3
    dist = [0.12, 0.05, 0.0]
    speeds = [0.3, 0.2, 0.08]
    threshs = [0.08, 0.03, 0.03]
    waypoints = [None] * num_waypoints
    for i in range(0, num_waypoints):
        wpose = copy.deepcopy(grasp.pose)
        wpose.position.x = wpose.position.x - dist[i] * grasp.approach.x
        wpose.position.y = wpose.position.y - dist[i] * grasp.approach.y
        wpose.position.z = wpose.position.z - dist[i] * grasp.approach.z
        waypoints[i] = copy.deepcopy(wpose)    
        print "---- waypoint", i, "----"
        print waypoints[i].position
        if visualizer is not None:
            visualizer.update(wpose)

    # move arm to waypoints
    ik_failed = False
    for i in range(0, num_waypoints):
        response = callBuiltInIK(waypoints[i].position, waypoints[i].orientation)
        if response.isValid:
            # import pdb; pdb.set_trace()
            solution = dict(zip(response.joints[0].name, response.joints[0].position))
            # Move to the first solution
            #s = raw_input("Press enter to move to waypoint {0}".format(i))
            print "Moving to waypoint {0}...".format(i),
            moveTo(solution, speeds[i])
            print "done."
        else:
            # No valid solutions for this waypoint, fail
            print "IK solver didn't return any valid solutions for waypoint {0}".format(i)
            ik_failed = True
            break
    # go back to home position if IK failed
    if ik_failed:
        s = raw_input("Press enter to return to home position")
        homePosition()
        return False
    else:
        # close the robot hand
        print "Reached grasp target"
        left_hand.close()
        
        # move back to 2md viapoint
        waypoint_up = copy.deepcopy(grasp.pose)
        waypoint_up.position.x = waypoint_up.position.x - 0.02 * grasp.approach.x
        waypoint_up.position.y = waypoint_up.position.y - 0.02 * grasp.approach.y
        waypoint_up.position.z = waypoint_up.position.z - 0.02 * grasp.approach.z
        response = callBuiltInIK(waypoint_up.position, waypoint_up.orientation)
        if response.isValid:
            solution = dict(zip(response.joints[0].name, response.joints[0].position))
            print "Moving to waypoint_up ..."
            moveTo(solution, speeds[2])
            print "Closing hand again"
            left_hand.close()
            print "done."
        
        #TODO this is hackey, we don't know that we made it to waypoint_up
        waypoint_lift = copy.deepcopy(waypoint_up)
        waypoint_lift.position.z = waypoint_lift.position.z + 0.1
        response = callBuiltInIK(waypoint_lift.position, waypoint_lift.orientation)
        if response.isValid:
            solution = dict(zip(response.joints[0].name, response.joints[0].position))
            print "Moving to waypoint_up ..."
            moveTo(solution, speeds[2])
            print "done."
        
        else:
            # No valid solutions for this waypoint, fail
            print "IK solver didn't return any valid solutions for waypoint_up"
            s = raw_input("Press enter to return to home position")
            homePosition()
            return False
        
        return True

class DoneException(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
         return repr(self.value)
     
def blockDone(msg):
     s = raw_input(msg)
     if s == 'q':
         raise DoneException("Quit from {0}".format(msg))
     
if __name__ == '__main__':
    
    rospy.init_node("single_cam_demo")
    # wait for ROS service for grasp selection
    # rospy.wait_for_service('/select_grasps/select_grasps')
    # select_grasps = rospy.ServiceProxy('/select_grasps/select_grasps', SelectGrasps)
    
    # Use the service that provides grasps that are easy to get to
    rospy.wait_for_service('/easy_grasps')
    select_grasps = rospy.ServiceProxy('/easy_grasps', SelectGrasps)
    
    #For publishing markers so rviz can see what is going on
    target_pub = rospy.Publisher('target', Marker, queue_size=1)
    wayViz = WaypointViz()
    
    # Enable the robot
    rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    try:
        rs.enable()
    except Exception, e:
        rospy.logerr(e.strerror)
    # initialize robot hand
    left_hand = baxter_interface.Gripper('left', baxter_interface.CHECK_VERSION)
    # calibrate robot hand
    if not left_hand.calibrated():
        instr = raw_input("Calibrate gripper (Y/n): ")
        if instr == "y" or instr == "Y":
            left_hand.calibrate()
    # open robot hand
    left_hand.open(block=True)
    
    # Set up to get the pose of the left arm later on
    limb = baxter_interface.Limb('left')

    attempts = 0
    successes = 0
    
    try:
        while not rospy.is_shutdown():
            # Wait for user to allow move to home position
            homePosition()
            blockDone("Reset object and hit enter when ready (q to quit)")
            
            call_failed = True
            while(call_failed):
                call_failed = False
                try:
                    # get the current pose of the robot
                    pose = limb.endpoint_pose() 
                    # convert to a geometry_msg
                    pose = Pose(pose['position'], pose['orientation'])
                    # Get the poses
                    response = select_grasps(pose)
                    # If no exception was thrown, we got an object, go pick it up
                    # Try to get a grasp until we actually get one
                    if len(response.grasps.grasps) > 0:
                        #Making an attempt
                        attempts += 1
                        #Publish visualization
                        target_pub.publish(createTargetMarker(response, 0))
                        #Move to the middle positon
                        homePosition()
                        #Attempt the grasp               
                        if grab_object(response.grasps.grasps[0], wayViz):
                            #Return to the middle position
                            homePosition()
                            #Query if the grasp was a success
                            gotValidAnswer = False
                            while not gotValidAnswer:
                                checkSuccess = raw_input("Was this attempt a success ")
                                if checkSuccess.lower() == 'y':
                                    successes += 1
                                    gotValidAnswer = True
                                if checkSuccess.lower() == 'n':
                                    gotValidAnswer = True
                            #Drop the object        
                            left_hand.open(block=True)
                    else:
                        print "Got a response with no grasps in it"
                except rospy.service.ServiceException as e:
                    rospy.logerr(e.message)
                    call_failed = True
    except DoneException as e:
        print e.value
    
    percentSuccess = (float(successes)/float(attempts))*100    
    print "Attempted {0} grasps, succeeded on {1} grasps, {2}% rate".format(attempts, successes, percentSuccess)
    
    
