#!/usr/bin/python

# Control the Baxter arm by continuiously moving the arm as long as one of the controls is held down
#
# Controls for cartesian coordinates are:
# U - increase z
# J - decrease z
# I - increase y
# K - decrease y
# O - increase z
# L - decrease z
#
# End effector angle controls are:
# Q - increase roll
# A - decrease roll
# W - increase pitch
# S - decrease pitch
# E - increase yaw
# D - decrease yaw
#
# Spacebar toggles gripper state (open/closed)
# Escape shuts everything down and closes this

import pygame
from pygame.locals import *
import sys
import rospy
import tf
import baxter_interface
from baxter_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest
)
from std_msgs.msg import Header
from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
)

class Axis:
	X, Y, Z, ROLL, PITCH, YAW = range(6)

class CartesianMover:
	def __init__(self):
		self.arm = baxter_interface.Limb("left")
		#Set the arm speed, 0.1 is pretty slow
		self.arm.set_joint_position_speed(0.3)
		self.hand = baxter_interface.Gripper('left', baxter_interface.CHECK_VERSION)
		if not self.hand.calibrated():
			self.hand.calibrate()
		# open robot hand
		self.hand.open(block=True)

		self.angles = None
		self.motions = {}

		#Get a service proxy for IK service requests
		self.ik_solver = None
		rospy.wait_for_service('ExternalTools/left/PositionKinematicsNode/IKService')
		try:
			self.ik_solver = rospy.ServiceProxy('ExternalTools/left/PositionKinematicsNode/IKService', SolvePositionIK, persistent=True)
		except rospy.ServiceException, e:
			print "Failed to get IK solver in init()"

	def doneMoving(self):
		return False

	#Given a pose (position/orientation), start moving towards it with each call of update()
	def setMoveTarget(self, targetPoint):
		# print("{0},{1},{2},{3},{4},{5},{6}".format(targetPoint.position.x,
		#  	targetPoint.position.y, targetPoint.position.z, targetPoint.orientation.x,
		#  	targetPoint.orientation.y, targetPoint.orientation.z, targetPoint.orientation.w ))
		response = None

		# Try calling the built-in IK
		if self.ik_solver is not None:
			req = SolvePositionIKRequest()
			req_head = Header(stamp=rospy.Time.now(), frame_id='base')
			req_pose = PoseStamped(header=req_head, pose=targetPoint)
			req.pose_stamp.append(req_pose)
			try:
				response = self.ik_solver(req)
			except rospy.ServiceException, e:
				print "IK service call failed"
				#TODO handle this error, although it almost never happens

			#Check the response and do it
			if response.isValid:
				#Get the angles and make those the new target angles
				solution = dict(zip(response.joints[0].name, response.joints[0].position))
				print solution
				self.angles = solution
			else:
				# No valid solutions for this waypoint, fail and stop the arm
				print "No IK solution, stopping"
				self.allStop()
		else:
			print "Didn't get IK solver in init()!"

	def allStop(self):
		#No motions is stopped
		self.motions = {}
		self.moving = False
		self.update()

	#Update has to be called regularly to keep the motion going
	def update(self):
		#Get the current end effector pose. This is full of immutable named tuples, so everything
		#has to be copied out of it, modified, and put back into a new Pose
		oldPose = self.arm.endpoint_pose()
		#l as in "linear"
		next_lx = oldPose['position'].x
		next_ly = oldPose['position'].y
		next_lz = oldPose['position'].z
		#a as in "angular"
		next_ax = oldPose['orientation'].x
		next_ay = oldPose['orientation'].y
		next_az = oldPose['orientation'].z
		next_aw = oldPose['orientation'].x

		#Convert to roll/pitch/yaw instead of a quaternion
		#print "Before ({0}, {1}, {2}, {3})".format(next_ax, next_ay, next_az, next_aw)
		#next_r, next_p, next_y = tf.transformations.euler_from_quaternion([next_ax, next_ay, next_az, next_aw])

		#Modify it based on the axes and rates in the motions dictionary
		#Amount to modify by, in meters. The speed of the motion is controllable
		#by self.arm.set_joint_position_speed(0.1), but I'm not doing that yet. 
		#Linear (meters) and Angular (radians) changes in position
		deltaL = 0.09
		deltaA = 0.02
		rpyChange = [0,0,0]
		for axis in self.motions.keys():
			if self.motions[axis] > 0:
				#Increasing motion along an axis, add a value
				if axis == Axis.X:
					next_lx += deltaL
				elif axis == Axis.Y:
					next_ly += deltaL
				elif axis == Axis.Z:
					next_lz += deltaL
				elif axis == Axis.ROLL:
				 	rpyChange[0] += deltaA
				elif axis == Axis.PITCH:
					rpyChange[1] += deltaA
				elif axis == Axis.YAW:
					rpyChange[2] += deltaA
			elif self.motions[axis] < 0:
				#Decreasing motion along an axis, subtract a value
				if axis == Axis.X:
					next_lx -= deltaL
				elif axis == Axis.Y:
					next_ly -= deltaL
				elif axis == Axis.Z:
					next_lz -= deltaL
				elif axis == Axis.ROLL:
				 	rpyChange[0] -= deltaA
				elif axis == Axis.PITCH:
					rpyChange[1] -= deltaA
				elif axis == Axis.YAW:
					rpyChange[2] -= deltaA
			else:
				#No motion on this axis, so don't change it
				pass
			
		#Convert the RPY change into a quaternion
		qd = tf.transformations.quaternion_from_euler(rpyChange[0], rpyChange[1], rpyChange[2])
		
		#Combine the  RPY change quaternion with the existing quaternion 
		#Note that quaternion multiplication is intransitive
		#next_ax, next_ay, next_az, next_aw = tf.transformations.quaternion_multiply([next_ax, next_ay, next_az, next_aw], qd)
		
		#Compose a new postition from the modifed values of the old one
		nextPosition = Point(x=next_lx, y=next_ly, z=next_lz)
		nextOrientation = Quaternion(x=next_ax, y=next_ay, z=next_az, w=next_aw)
		nextPose = Pose(orientation=nextOrientation, position = nextPosition)
		#Convert the next pose to a set of joint angles and start moving towards it
		self.setMoveTarget(nextPose)
		if self.angles is not None:
			#Move towards the requested position a bit
			if not self.doneMoving():
				self.arm.set_joint_positions(self.angles)

	#TODO these are both glorified setters at this point, could consolidate code
	def linearMove(self, axis, rate):
		self.motions[axis] = rate

	def rotationMove(self, axis, rate):
		self.motions[axis] = rate

	def toggleGripper(self):
		if self.hand.position() > 95:
			#gripper is open, so close it. Can be confused by a big object, though. 
			self.hand.close()
		elif self.hand.gripping() or self.hand.position() < 5:
			#gripper is closed on something, or on nothing
			self.hand.open()
		else:
			#Unknown state, so open the gripper
			self.hand.open()

class InterfaceHandler:
	def __init__(self, mover, width = 320, height = 240):
		pygame.init()
		scr = pygame.display.set_mode((width, height))
		self.isDone = False
		#Keys, the directions of motion, and the axes of motion for linear moves
		self.linearKeys = [K_u, K_j, K_i, K_k, K_o, K_l]
		self.linearDirs = [1, -1, 1, -1, 1, -1]
		self.linearAxes = [Axis.X, Axis.X, Axis.Y, Axis.Y, Axis.Z, Axis.Z]
		#Same as above, but for rotation
		self.angularKeys = [K_q, K_a, K_w, K_s, K_e, K_d]
		self.angularDirs = [1, -1, 1, -1, 1, -1]
		self.angularAxes = [Axis.ROLL, Axis.ROLL, Axis.PITCH, Axis.PITCH, Axis.YAW, Axis.YAW]
		#The other control keys
		self.otherKeys = [K_SPACE, K_ESCAPE]
		self.mover = mover

	#Motion rates are +/-1, with 0 being stopped and 1 being full speed
	def linearMove(self, key):
		#Get the index of this key in the linear keys
		idx = self.linearKeys.index(key)
		#Look up the axis to move on and the direction
		axis = self.linearAxes[idx]
		move = self.linearDirs[idx]
		#TODO if you wanted to multiply the direction by a rate to get a speed/dir, this would be a good spot
		#Start the motion
		self.mover.linearMove(axis, move)	

	# TODO could tighten this up by putting linear and rotational keys together in one array
	def rotationalMove(self, key):
		idx = self.angularKeys.index(key)
		axis = self.angularAxes[idx]
		move = self.angularDirs[idx]
		self.mover.rotationMove(axis, move)

	def stopLinear(self, key):
		idx = self.linearKeys.index(key)
		self.mover.linearMove(self.linearAxes[idx], 0)

	def stopRotation(self, key):
		idx = self.angularKeys.index(key)
		self.mover.rotationMove(self.angularAxes[idx], 0)
	
	def handleKeyDown(self, key):
		#Handle escape key
		if key == K_ESCAPE:
			self.isDone = True
		elif key in self.linearKeys:
			self.linearMove(key)
		elif key in self.angularKeys:
			self.rotationalMove(key)
		elif key == K_SPACE:
			self.mover.toggleGripper()
		else:
			print "Pressed {0}, didn't know what to do about it".format(key)

	def handleKeyUp(self, key):
		if key in self.linearKeys:
			self.stopLinear(key)
		elif key in self.angularKeys:
			self.stopRotation(key)
		elif key in self.otherKeys:
			#It should have been handled in handleKeyDown
			pass 
		else:
			print "Released {0}, didn't know what to do about it".format(key)

	def mainLoop(self):
		#Main event handling loop
		while not self.isDone:
			#Check that the window is focused
			if pygame.key.get_focused() and any(pygame.key.get_pressed()):
				#Update the mover regularly to ensure smooth motion.
				#Since this is only done when the window has focus, defocusing will stop the robot
				#It is also only done if there is a pressed key
				self.mover.update()
			#Event handling for pygame events
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					self.isDone = True
				if event.type == pygame.KEYDOWN:
					#Handle key being pressed
					self.handleKeyDown(event.key)
				if event.type == pygame.KEYUP:
					#Handle key being released
					self.handleKeyUp(event.key)

		#TODO could put cleanup code here, if needed

if __name__=="__main__":
	#Start rospy for talking to the robot
	rospy.init_node("keyboard_arm_teleop")

	# Enable the robot
	rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
	try:
		rs.enable()
	except Exception, e:
		rospy.logerr(e.strerror)
	

	#Mover, actually talks to the robot
	mover = CartesianMover()
	#Gets keypresses and talks to the mover
	ih = InterfaceHandler(mover)

	#Kick off the main loop. This doesn't return until the program ends
	ih.mainLoop()

