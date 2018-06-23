#!/usr/bin/python

from math import cos, sin

import numpy # scipy
from numpy.linalg import norm, pinv # scipy
from numpy import array, dot # scipy

import pygame # pygame
from pygame.locals import * # pygame

import openravepy # openrave

import rospy # ros
import sensor_msgs.msg as sensor_msgs # ros

import baxter_interface # rethink
from baxter_core_msgs.msg import JointCommand # rethink

class CartesianMover:
  '''Arm control for keyboard commands'''
  
  def __init__(self, armName, isMoving, useViewer):
    
    self.armName = armName
    self.isMoving = isMoving
    
    self.configHome = array([1.03198, -1.73454, 0.51656, 1.50521, 0.02224, 1.75564, -0.00076])
    self.configDrop = array([1.22181, -1.825053, 1.693898,  1.178864, 0.30372, 2.083145, -0.182160])
    
    # Initialize OpenRAVE.
    
    self.env = openravepy.Environment()
    self.env.StopSimulation()
    openraveRoot = '/home/csrobot/Code/arm_ws/src/active_sensing/openrave/'
    self.env.Load(openraveRoot + "baxter_ik.xml")
    self.env.Load(openraveRoot + "table.xml")
    self.robot = self.env.GetRobots()[0]
    self.manip = self.robot.SetActiveManipulator(armName+'_arm')
    if useViewer: self.env.SetViewer('qtcoin') # start the viewer (cconflicts with matplotlib)
    
    # Calibrate and open gripper.
    
    self.baxterHand = baxter_interface.Gripper(armName, baxter_interface.CHECK_VERSION)
    if not self.baxterHand.calibrated():
      self.baxterHand.calibrate()
    self.baxterHand.open(block=True)
    self.baxterHandOpen = True
    
    # Start subscribers.
    
    self.joint_values = [0] * 7
    self.joint_indices = [9,10,11,12,13,14,15]
    
    self.joints_sub = rospy.Subscriber("/robot/joint_states", sensor_msgs.JointState, self.jointsCallback)
    self.pub = rospy.Publisher("/robot/limb/"+armName+"/joint_command", JointCommand, queue_size=10)
    
    rospy.sleep(2.0)
  
  def getClosestSolution(self, configs1, configs2):
    '''Returns the pair of solutions that are closest.'''
    
    dBest = float('inf'); c1Best = None; c2Best = None
    for c1 in configs1:
      for c2 in configs2:
        v = c1-c2
        d = dot(v,v)
        if d < dBest: c1Best = c1; c2Best = c2; dBest = d
    
    return (c1Best, c2Best)
  
  def jointsCallback(self, msg):
    '''Callback function for the joint_states ROS topic.'''
    
    joint_values = msg.position[0 : 7]
    self.joint_names = msg.name[0 : 7]
    
    # ordering of jointstates topic is different from IK solver ordering
    self.joint_values[0] = joint_values[2]
    self.joint_values[1] = joint_values[3]
    self.joint_values[2] = joint_values[0]
    self.joint_values[3] = joint_values[1]
    self.joint_values[4:8] = joint_values[4:8] 
  
  def moveBack(self):
    '''Use trajopt to plan a trajectory moving back to the basket. Returns finished flag.'''
    
    step_size = 2.0; speed = 0.5; thresh = 0.07
    
    # calculate joint error, error magnitude, and command velocity
    joint_error = self.configDrop - self.joint_values
    joint_velocity_command = joint_error * step_size
    error_magnitude = numpy.linalg.norm(joint_error)
    joint_velocity_command_magnitude = error_magnitude * step_size
    
    # if command velocity magnitude exceeds speed, scale it back to speed
    if joint_velocity_command_magnitude > speed:
      joint_velocity_command = speed * joint_velocity_command / joint_velocity_command_magnitude
    
    # send velocity command to joints
    values_out = [joint_velocity_command[2], joint_velocity_command[3], joint_velocity_command[0], 
      joint_velocity_command[1], joint_velocity_command[4], joint_velocity_command[5], joint_velocity_command[6]]
    self.pub.publish(JointCommand(JointCommand.VELOCITY_MODE, values_out, self.joint_names))
    
    # break loop if close enough to target joint values
    if error_magnitude < thresh:
      self.releaseObject()
      return True
      
    return False
  
  def moveVelocity(self, jointTarget, speed=0.5, thresh=0.07):
    '''Block while moving arm to joint target.'''
    
    max_iterations = 10000; step_size = 2.0
  
    for i in range(0, max_iterations):
      joint_error = array(self.joint_values)
      joint_velocity_command = joint_error
    
      # calculate joint error, error magnitude, and command velocity
      joint_error = jointTarget - joint_error
      joint_velocity_command = joint_error * step_size
      error_magnitude = numpy.linalg.norm(joint_error)
      joint_velocity_command_magnitude = error_magnitude * step_size
      
      # if command velocity magnitude exceeds speed, scale it back to speed
      if joint_velocity_command_magnitude > speed:
        joint_velocity_command = speed * joint_velocity_command / joint_velocity_command_magnitude
      
      # send velocity command to joints
      values_out = [joint_velocity_command[2], joint_velocity_command[3], joint_velocity_command[0], 
        joint_velocity_command[1], joint_velocity_command[4], joint_velocity_command[5], joint_velocity_command[6]]
      self.pub.publish(JointCommand(JointCommand.VELOCITY_MODE, values_out, self.joint_names))
      
      # break loop if close enough to target joint values
      if error_magnitude < thresh:
        break
      
      rospy.sleep(0.001)
  
  def releaseObject(self):
    '''Goes down, opens gripper, and moves home.'''
    
    # update state in OpenRAVE
    self.robot.SetDOFValues(self.joint_values, self.manip.GetArmIndices())
    self.env.UpdatePublishedBodies()    
    
    # move down
    T = self.manip.GetEndEffectorTransform()
    T[2,3] = T[2,3] - 0.15
    
    solutions = self.manip.FindIKSolutions(T, openravepy.IkFilterOptions.CheckEnvCollisions)
    joint_values, solution = self.getClosestSolution([self.joint_values], solutions)
    
    if solution is not None:
      self.moveVelocity(solution, speed=0.2)
    
    # release object
    self.baxterHand.open()
    self.baxterHandOpen = True
    rospy.sleep(1.0)
    
    # move home
    self.moveVelocity(self.configHome, speed=0.5)
  
  def toggleGripper(self):
    '''Opens/closes the gripper.'''
    
    if self.baxterHandOpen:
      self.baxterHand.close()
    else:
      self.baxterHand.open()
    self.baxterHandOpen = not self.baxterHandOpen
    print("It is {} that the hand is open.".format(self.baxterHandOpen))
    
  def update(self, cmd, cmdIsAxisAngle, speed):
    '''Run controller.'''
    
    # update arm position in OpenRAVE
    self.robot.SetDOFValues(self.joint_values, self.manip.GetArmIndices())
    self.env.UpdatePublishedBodies()    
    
    JI = pinv(self.manip.CalculateJacobian())
    jointVelocities = speed * dot(JI, cmd[0:3])
    
    if cmdIsAxisAngle:
      JIw = pinv(self.manip.CalculateAngularVelocityJacobian())
      jointVelocities = jointVelocities + 2 * speed * dot(JIw, cmd[3:])
    else:
      jointVelocities[0] = jointVelocities[0] + 2*speed*cmd[3]
      jointVelocities[5] = jointVelocities[5] + 3*speed*cmd[4]
      jointVelocities[6] = jointVelocities[6] + 4*speed*cmd[5]
    
    #print jointVelocities  
    
    if self.isMoving:
      values_out = [jointVelocities[2], jointVelocities[3], jointVelocities[0], jointVelocities[1], jointVelocities[4],
        jointVelocities[5], jointVelocities[6]]
      self.pub.publish(JointCommand(JointCommand.VELOCITY_MODE, values_out, self.joint_names))

class InterfaceHandler:
  
  def __init__(self, mover, frameRate, width=320, height=320):
    
    self.frameRate = frameRate
    self.mover = mover
    
    # init pygame
    pygame.init()
    pygame.display.set_mode((width, height))
    self.isDone = False
    
  def mainLoop(self):
    '''Main event handling loop.'''
    
    movingBack = False; axisAngle = False
    speed = 0.10; minSpeed = 0.01; maxSpeed = 1.0
    
    spaceCount = 0; minusCount = 0; equalsCount = 0; enterCount = 0; mCount = 0
    while not self.isDone and not rospy.is_shutdown():
      
      spaceCount -= 1; minusCount -= 1; equalsCount -= 1; enterCount -= 1; mCount -= 1
      rospy.sleep(1/self.frameRate)
      pygame.event.pump()
      keys = pygame.key.get_pressed()      
      
      if keys[K_ESCAPE]:
        self.isDone = True
        
      if spaceCount <= 0 and keys[K_SPACE]:
        mover.toggleGripper()
        spaceCount = self.frameRate
      
      if minusCount <= 0 and keys[K_MINUS]:
        print("Decreased speed.")
        speed = max(speed-0.01, minSpeed)
        minusCount = self.frameRate/4
        
      if equalsCount <= 0 and keys[K_EQUALS]:
        print("Increased speed.")
        speed = min(speed+0.01, maxSpeed)
        equalsCount = self.frameRate/4
      
      if mCount <=0 and keys[K_m]:
        axisAngle = not axisAngle
        if axisAngle: print("Now using axis angle for rotation.")
        else: print("Now controlling joints for rotation.")
        mCount = 2*self.frameRate
      
      if enterCount <= 0 and keys[K_RETURN]:
        movingBack = not movingBack
        print("Moving back is {}.".format(movingBack))
        enterCount = 2*self.frameRate
      
      # +/- x,y,z,r,p,y
      command = array([\
        keys[K_w]-keys[K_s], \
        keys[K_a]-keys[K_d], \
        keys[K_q]-keys[K_e], \
        keys[K_LEFT]-keys[K_RIGHT], \
        keys[K_DOWN]-keys[K_UP], \
        keys[K_PERIOD]-keys[K_COMMA] \
        ])
      
      if not movingBack:
        self.mover.update(command, axisAngle, speed)
      else:
        movingBack = not self.mover.moveBack()
        if not movingBack:
          print("Arrived back home. You have control again.")
      

if __name__=="__main__":
  #Start rospy for talking to the robot
  rospy.init_node("user_keyboard_control")

  # Enable the robot
  #rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
  #try:
  #  rs.enable()
  #except Exception, e:
  #  rospy.logerr(e.strerror)

  #Mover, actually talks to the robot
  mover = CartesianMover("left", True, False)
  #Gets keypresses and talks to the mover
  ih = InterfaceHandler(mover, 1000)

  #Kick off the main loop. This doesn't return until the program ends
  ih.mainLoop()
