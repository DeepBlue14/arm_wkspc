#!/usr/bin/env python
"""Baxter End Effector Controller Demo by Jackie Kay
Based on the Baxter RSDK Joint Position Example

Command the (x, y, z) position of the end effector using the keyboard. Uses
Jacobian kinematics to determine required joint angles. 
"""

# TODO:
# figure out coordinate frame and rotational frame: convert increment from world frame to
# robot frame

import argparse
import struct
import rospy
import tf

import ik_command 

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

from baxter_pykdl import baxter_kinematics

from operator import add

import numpy as np
from math import atan2, asin


# Convert the last 4 entries in q from quaternion form to Euler Angle form and copy into p
# Expect a 7x1 column vector and a preallocated 6x1 column vector (numpy)
# p and q are both pose vectors
def quaternion_to_euler(q, p):
    if q.shape != (7, 1) or p.shape != (6, 1):
        raise Exception("Got unexpected vector dimension in quaternion_to_euler")
    p[0:3] = q[0:3]
    p[3], p[4], p[5] = tf.transformations.euler_from_quaternion(q[3:7].flatten().tolist())

#Rotation of the wrist as roll, pitch, yaw. 
desired_angles = [0,0,0]

#Increments used for rotation and translation of gripper
inc = 0.1
angle_inc = 0.1

# Get key commands from the user and move the end effector
def map_keyboard():
    global desired_angles, inc, angle_inc
    left = baxter_interface.Limb('left')    
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)qqqqqqqqqqqa
    
    #These are from PyKDL and are needed for the Jacobian
    left_kin = baxter_kinematics('left')
    
    #Connect with IK service    
    left_iksvc, left_ns = ik_command.connect_service('left')
    
    #Initialize desired angles from current angles
    init_orient = left.endpoint_pose()['orientation']
    quat = [0,0,0,0]
    quat[3] = init_orient.w
    quat[0] = init_orient.x
    quat[1] = init_orient.y
    quat[2] = init_orient.z
    desired_angles = tf.transformations.euler_from_quaternion(quat)
    #print init_orient

    def command_jacobian(side, direction):
        #UNUSED
        if side == 'left':
            limb = left
            kin = left_kin        
        else:
            raise Exception("Got wrong side in command_jacobian")

        # current is the current position of end effector
        current_p = np.array(limb.endpoint_pose()['position']+limb.endpoint_pose()['orientation']) 
        current_p = current_q.reshape((7, 1))
        current = np.zeros((6, 1))
        quaternion_to_euler(current_p, current)

        # We need to convert the direction from the world frame to the hand frame
        # Rotate direction by the inverse of the rotation matrix from the angular pose in current
        R = tf.transformations.quaternion_matrix(current_p[3:7].flatten().tolist())
        R_inv = tf.transformations.inverse_matrix(R)
        print R_inv[0:3, 0:3]
        direction = np.array(direction).reshape((6, 1))
        print direction[0:3]
        dir_rot = np.dot(R_inv[0:3, 0:3], direction[0:3])
        print dir_rot
        #direction[0:3] = dir_rot

        # Goal is the goal position, found by adding the requested direction from the user
        goal = current + direction
        current_angles = np.array([limb.joint_angles()[name] for name in limb.joint_names()])

        # Get the Jacobian inverse and solve for the necessary change in joint angles
        jacobian_inv = kin.jacobian_transpose()
        delta = jacobian_inv*(goal-current)
        commands = current_angles.flatten() + delta.flatten()

        # Command the joints
        command_list = commands.tolist()
        joint_command = dict(zip(limb.joint_names(), command_list[0]))
        limb.set_joint_positions(joint_command)

    def command_ik(side, direction):
        """Use the Rethink IK service to figure out a desired joint position"""
        if side == 'left':
            limb = left
            iksvc = left_iksvc
            ns = left_ns            
        current_p = np.array(limb.endpoint_pose()['position']+limb.endpoint_pose()['orientation']) 
        direction = np.array(direction)
        print direction
        desired_p = current_p + direction
        ik_command.service_request(iksvc, desired_p, side)

    zeros = [0]*4
   
    def update_angles(change_angles):
      global desired_angles
      current_angles = np.array(desired_angles)
      change_angles = np.array(change_angles)
      #Note that adding np arrays does element-wise summation, but adding python lists concatenates them 
      new_angles = current_angles + change_angles
      #Check that the angles are in bounds, angles are in radians
     #for value in new_angles:
        #Wrap crossing zero towards 2*pi
      #  if value < 0:
      #    value = (2* np.pi) + value
        #Wrap crossing 2*pi towards zero
      #  if value > (2 * np.pi):
      #    value = value - (2*np.pi)
      #Convert to a python array and store in the desired angles
      desired_angles = new_angles.tolist()
      #Convert the desired angles to a quaternion
      quat = tf.transformations.quaternion_from_euler(desired_angles[0], desired_angles[1], desired_angles[2])
      #This only does the left side, because the scooter only has a left arm
      limb = left
      current_p = np.array(limb.endpoint_pose()['position']+limb.endpoint_pose()['orientation']) 
      current_p[3:7] = quat
      ik_command.service_request(left_iksvc, current_p, "left")
      
    def update_inc(isSmall = True):
        global inc, angle_inc
        if isSmall:
            inc = 0.1
            angle_inc = 0.1
        else:
            inc = 0.5
            angle_inc = 0.2

    def toggle_gripper():
        if grip_left.position() > 95:
            #gripper is open, so close it. Can be confused by a big object, though. 
            print "Closing gripper"
            grip_left.close()
        elif grip_left.gripping() or grip_left.position() < 5:
            print "Opening gripper"
            #gripper is closed on something, or on nothing
            grip_left.open()
        else:
            print "Unknown state, opening gripper"
            #Unknown state, so open the gripper
            grip_left.open()

    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        #This had to be moved into the loop so the increment could be rebound when 
        #the -/= keys are used to change the increment between small and large
        bindings = {        
            'u': (command_ik, ['left', [inc, 0, 0]+zeros],  "increase left x"),
            'j': (command_ik, ['left', [-inc, 0, 0]+zeros], "decrease left x"),
            'i': (command_ik, ['left', [0, inc, 0]+zeros],  "increase left y"),
            'k': (command_ik, ['left', [0, -inc, 0]+zeros], "decrease left y"),
            'o': (command_ik, ['left', [0, 0, inc]+zeros],  "increase left z"),
            'l': (command_ik, ['left', [0, 0, -inc]+zeros], "decrease left z"),
            'q': (update_angles, [[angle_inc, 0, 0]], "increase left roll"),
            'a': (update_angles, [[-angle_inc, 0, 0]], "decrease left roll"),
            'w': (update_angles, [[0,angle_inc,0]], "increase left pitch"),
            's': (update_angles, [[0,-angle_inc,0]], "decrease left pitch"),
            'e': (update_angles, [[0,0,angle_inc]], "increase left yaw"),
            'd': (update_angles, [[0,0,-angle_inc]], "decrease left yaw"),
            '-': (update_inc, [True], "small motion increments"),
            '=': (update_inc, [False], "large motion increments"),
            ' ': (toggle_gripper, [], "toggle gripper state")        
        }
        c = baxter_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Example finished.")
            elif c in bindings:
                cmd = bindings[c]
                #expand binding to something like "command_jacobian(left, [0.1, 0, 0])"
                cmd[0](*cmd[1])
                print("command: %s with params %s" % (cmd[2], cmd[1]))
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))



def main():
    """RSDK End Effector Position Example: Keyboard Control

    Use your dev machine's keyboard to control end effector positions.

    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    map_keyboard()
    print("Done.")


if __name__ == '__main__':
    main()

