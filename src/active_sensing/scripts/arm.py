''' 
This module represents a robot arm.
'''

# python
import copy
# scipy
import numpy
from numpy.linalg import norm
from numpy import array, ascontiguousarray, concatenate, pi, zeros
# ros
import rospy
import sensor_msgs.msg as sensor_msgs
# openrave
import openravepy
import cloudprocpy
import trajoptpy.kin_utils as ku
from trajoptpy import make_kinbodies
# baxter
import baxter_interface
from baxter_core_msgs.msg import JointCommand, EndpointState


class Arm:
  '''A class for moving the Baxter arm.'''
  
  def __init__(self, name, endEffector, viewEffector, isMoving, useRaveViewer):
    '''TODO'''
    
    self.name = name
    self.endEffector = endEffector
    self.viewEffector = viewEffector
    self.isMoving = isMoving
    
    # structure_collision_link, structure_link
    self.handLinkNames = set(["left_hand", "left_gripper", "hand_motor_link", "left_finger_link", \
      "right_finger_link", "structure_link"])
    #self.handLinkNames = set(["left_hand", "left_gripper", "hand_motor_link", "left_finger_link", \
    #  "right_finger_link", "structure_link", "structure_collision_link"])
    
    # initialize openrave
    self.env = openravepy.Environment()
    self.env.StopSimulation()
    openraveRoot = '/home/james/Code/arm_wkspc/src/active_sensing/openrave/'
    self.env.Load(openraveRoot + "baxter_ik.xml")
    self.env.Load(openraveRoot + "table.xml")
    self.robot = self.env.GetRobots()[0]
    self.manip = self.robot.SetActiveManipulator(name + "_arm")
    if useRaveViewer:
      # start the viewer (cconflicts with matplotlib) 
      self.env.SetViewer('qtcoin')    
    
    # initialize values set in callbacks
    self.forceMagnitude = 0.0
    self.joint_values = [0] * 7
    self.joint_velocities = [0] * 7
    self.joint_indices = [9,10,11,12,13,14,15] # from ROS topic /robot/joint_states
    self.obstacleCloud = None
    
    # set joint limits
    limits = self.robot.GetDOFLimits()
    limits[0][5] += 3*(pi/180) # w1 low
    limits[1][5] -= 3*(pi/180) # w1 high
    limits[0][3] += 5*(pi/180) # low
    limits[1][3] -= 5*(pi/180) # high
    self.robot.SetDOFLimits(limits[0], limits[1])
    self.joint_limits = self.robot.GetDOFLimits()
    
    # subscribe to robot state topics
    self.joints_sub = rospy.Subscriber("/robot/joint_states", sensor_msgs.JointState, self.jointsCallback)
    self.force_sub = rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, self.forceCallback)
    self.pub = rospy.Publisher('/robot/limb/left/joint_command', JointCommand, queue_size=10)
    
    # calibrate robot hand
    self.gripper = baxter_interface.Gripper(name, baxter_interface.CHECK_VERSION) 
    if self.isMoving and not self.gripper.calibrated():
      self.gripper.calibrate()
    if self.isMoving:
      self.gripper.set_holding_force(50)
      self.gripper.set_moving_force(50)
 
    
  def addCloudToEnvironment(self, cloud, cubeSize=0.02):
    '''
    Add a point cloud to the OpenRAVE environment as a collision obstacle.
    
    @param cloud: list of 3-tuples, [(x1,y1,z1),...,(xn,yn,zn)]
    '''
    
    # remove existing cloud, if there is one
    self.removeCloudFromEnvironment()
    
    # convert point cloud to expected format
    cloud = ascontiguousarray(cloud) # cloudprocpy assumes this
    cloud = concatenate((cloud, zeros((cloud.shape[0], 1))), axis=1)
    cloudTrajopt = cloudprocpy.CloudXYZ()
    cloudTrajopt.from2dArray(cloud)
    
    # downsample and add to environment
    dsCloud = cloudprocpy.downsampleCloud(cloudTrajopt, cubeSize)
    self.obstacleCloud = make_kinbodies.create_boxes(self.env, dsCloud.to2dArray()[:,:3], cubeSize/2.0)
    
  
  def addOldCloudToEnvironment(self):
    '''TODO'''
    
    self.env.Add(self.obstacleCloud)
  
  
  def calcFK(self, joint_positions, end_effector_name):
    '''
    Calculate the Cartesian pose for a given collection of joint positions.
    
    @type joint_positions: 1x7 vector
    @param joint_positions: the joint positions for which the pose is calculated
    @type end_effector_name: string
    @param end_effector_name: the name of the end-effector for which the pose is calculated
    '''
    self.robot.SetDOFValues(joint_positions, self.manip.GetArmIndices())
    return self.robot.GetLink(end_effector_name).GetTransform()
  
  
  def calcIKForPQ(self, pos, quat, opts=openravepy.IkFilterOptions.CheckEnvCollisions):
    '''TODO'''
    targetMat = openravepy.matrixFromPose(numpy.r_[quat, pos])
    solutions = self.manip.FindIKSolutions(targetMat, opts)
    return solutions
  
  
  def calcIKForT(self, T, opts=openravepy.IkFilterOptions.CheckEnvCollisions):
    '''TODO'''
    
    return self.manip.FindIKSolutions(T, opts)
  
    
  def calcIKForTGivenLink(self, T, linkName, opts=openravepy.IkFilterOptions.CheckEnvCollisions):
    '''TODO'''
    
    return ku.ik_for_link(T, self.manip, linkName, return_all_solns=True, filter_options=opts)
  
  
  def calcIKForTIgnoreHandCollisions(self, T):
    '''TODO'''
    
    solutions = self.calcIKForT(T, 0)
    
    freeSolutions = []
    for solution in solutions:
      if not self.checkCollisionIgnoreHand(solution):
        freeSolutions.append(solution)
    
    return freeSolutions
  
  
  def calcIKWithNoise(self, T, sigma, nAttempts, opts=openravepy.IkFilterOptions.CheckEnvCollisions):
    '''TODO'''
    
    positionNoise = numpy.random.normal(0, sigma, (nAttempts,3)).tolist()
    positionNoise[0] = zeros(3)
    for noise in positionNoise:
      NT = copy(T)
      NT[0:3,3] = NT[0:3,3] + noise
      configs = self.calcIKForT(NT, opts)
      if len(configs) > 0: break
    
    return configs
  
  
  def calcIKForPoseMsg(self, poseMsg, opts):
    '''
    Calculate the inverse kinematics solutions for a given ROS pose message.
    
    @type poseMsg: geometry_msgs/Pose
    @param poseMsg: the given pose message
    @type opts: bit mask
    @param opts: collision checker options for the inverse kinematics solver    
    '''
    pos = array([poseMsg.position.x, poseMsg.position.y, poseMsg.position.z])
    quat = array([poseMsg.orientation.w, poseMsg.orientation.x, poseMsg.orientation.y, poseMsg.orientation.z])
    return self.calcIK(pos, quat)  
  
  
  def checkCollisionIgnoreHand(self, config):
    '''TODO'''
    
    self.robot.SetDOFValues(config, self.manip.GetArmIndices())
    self.env.UpdatePublishedBodies()
    report = openravepy.CollisionReport()
    inCollision = self.env.CheckCollision(self.robot, report)
    
    if not inCollision: return False    
    
    inCollision = False
    for linkPair in report.vLinkColliding:
      if not (linkPair[0].GetName() in self.handLinkNames or linkPair[1].GetName() in self.handLinkNames):
        inCollision = True
        break
    
    return inCollision
  
  
  def closeGripper(self):
    '''TODO'''
    if self.isMoving:
      self.gripper.close(block=True)
  
  
  def findClosestIK(self, solutions):
    '''
    Find the joint positions closest to the current joint positions.
    
    @type solutions: list of 1x7 vectors
    @param solutions: the list of joint positions
    '''
    closestSolution = None
    minDist = float('inf')
    for solution in solutions:
      dist = numpy.linalg.norm(self.joint_values - solution)
      if dist < minDist:
        minDist = dist
        closestSolution = solution
    return closestSolution
  
  
  def findClosestIKDistance(self, solutions):
    '''
    Find the joint positions closest to the current joint positions, and return its distance.
    
    @type solutions: list of 1x7 vectors
    @param solutions: the list of joint positions
    @rtype: real number
    @return: the distance to the closest joint positions in the given list
    '''
    minDist = float('inf')
    for solution in solutions:
      dist = numpy.linalg.norm(self.joint_values - solution)
      if dist < minDist:
        minDist = dist
    return minDist
  
  
  def followTrajectory(self, traj, startSpeed=0.75, endSpeed=0.25, startThresh=0.30, \
    endThresh=0.10, forceThresh=float('inf')):
    '''TODO'''
    
    if not self.isMoving:
      return
      
    for i in xrange(len(traj)):
      if i == len(traj)-1:
        self.moveVelocity(traj[i], endSpeed, endThresh, forceThresh=forceThresh)
      if i == len(traj)-2:
        self.moveVelocity(traj[i], 0.5*startSpeed + 0.5*endSpeed, startThresh, forceThresh=forceThresh)
      else:
        self.moveVelocity(traj[i], startSpeed, startThresh, forceThresh=forceThresh)
    
    # send a zero velocity command at the end of the trajectory
    self.pub.publish(JointCommand(JointCommand.VELOCITY_MODE, [0]*7, self.joint_names))
  
  
  def forceCallback(self, msg):
    '''TODO'''
    
    forceMsg = msg.wrench.force
    self.forceMagnitude = forceMsg.x**2 + forceMsg.y**2 + forceMsg.z**2  
  
  
  def isInCollision(self, joint_positions):
    '''
    Check whether the robot is in collision for the given joint positions.
    
    @type joint_positions: 1x7 vector
    @param joint_positions: the joint positions for which collisions are checked
    '''    
    self.robot.SetDOFValues(joint_positions, self.manip.GetArmIndices())
    self.env.UpdatePublishedBodies()
    return self.env.CheckCollision(self.robot)  
  
  
  def isInJointLimits(self, joint_positions):
    '''TODO'''
    
    return (joint_positions >= self.joint_limits[0]).all() and (joint_positions <= self.joint_limits[1]).all()  
  
  
  def jointsCallback(self, msg):
    ''' 
    Callback function for the joint_states ROS topic.
    
    @type msg: sensor_msgs/JointState
    @param msg: the incoming message from the joint_states topic    
    '''
    
    joint_values = msg.position[0 : 7]
    joint_velocities = msg.velocity[0 : 7]
    self.joint_names = msg.name[0 : 7]
    
    # ordering of jointstates topic is different from IK solver ordering
    self.joint_values[0] = joint_values[2]
    self.joint_values[1] = joint_values[3]
    self.joint_values[2] = joint_values[0]
    self.joint_values[3] = joint_values[1]
    self.joint_values[4:8] = joint_values[4:8]
    
    self.joint_velocities[0] = joint_velocities[2]
    self.joint_velocities[1] = joint_velocities[3]
    self.joint_velocities[2] = joint_velocities[0]
    self.joint_velocities[3] = joint_velocities[1]
    self.joint_velocities[4:8] = joint_velocities[4:8]

  
  def moveVelocity(self, joint_values_target, speed, thresh, \
    forceThresh=float('inf'), maxAcceleration=0.2, is_printing=False):
    '''
    Move the arm using velocity control to a given list of joint positions.
    
    @type jointTarget: 1x7 vector
    @param jointTarget: the given joint configuration
    @type speed: real number
    @param speed: the speed at which the arm should move
    @type thresh: real number 
    @param thresh: the threshold at which the arm is close enough to the desired joint positions
    @type is_printing: bool
    @param is_printing: true if debug information should be printed, false otherwise  
    '''
    
    if not self.isMoving:
      return
    
    max_iterations = 10000
    step_size = 2.0
    
    oldVelocityCommand = array(self.joint_velocities)
    
    for i in range(0, max_iterations):
      joint_error = array(self.joint_values)
      joint_velocity_command = joint_error
    
      # calculate joint error, error magnitude, and command velocity
      joint_error = joint_values_target - joint_error
      joint_velocity_command = joint_error * step_size
      error_magnitude = numpy.linalg.norm(joint_error)
      joint_velocity_command_magnitude = error_magnitude * step_size
      
      # if command velocity magnitude exceeds speed, scale it back to speed
      if joint_velocity_command_magnitude > speed:
        joint_velocity_command = speed * joint_velocity_command / joint_velocity_command_magnitude
      
      # scale back if over acceleration limits
      acceleration = norm(joint_velocity_command - oldVelocityCommand)
      if acceleration > maxAcceleration:
        #print("Exceeded max acceleration: {}.".format(acceleration))
        a = (maxAcceleration/acceleration)
        joint_velocity_command = a * joint_velocity_command + (1.0 - a) * oldVelocityCommand
      oldVelocityCommand = joint_velocity_command
      
      # exit if force magnitude exceeded
      if self.forceMagnitude > forceThresh:
        print("Force is {}, exceeded threshold {}. Stopping.".format(self.forceMagnitude, forceThresh))
        break      
      
      # send velocity command to joints
      values_out = [joint_velocity_command[2], joint_velocity_command[3], joint_velocity_command[0], 
        joint_velocity_command[1], joint_velocity_command[4], joint_velocity_command[5], joint_velocity_command[6]]
      self.pub.publish(JointCommand(JointCommand.VELOCITY_MODE, values_out, self.joint_names))
      
      if is_printing:
        print "iteration:", i, "error_magnitude: ", error_magnitude
      
      # break loop if close enough to target joint values
      if error_magnitude < thresh:
        break
      
      rospy.sleep(0.01)
  
  
  def openGripper(self):
    '''TODO'''
    if self.isMoving:
      self.gripper.open(block=True)  
  
  
  def removeCloudFromEnvironment(self):
    '''
    Remove any existing point cloud obstacle from the environment.    
    '''
    
    if self.obstacleCloud != None:
      self.env.Remove(self.obstacleCloud)
      
  
  def update(self):
    ''' 
    Set the OpenRave robot to the current joint values and update the viewer.
    '''
    self.robot.SetDOFValues(self.joint_values, self.manip.GetArmIndices())
    self.env.UpdatePublishedBodies()
    rospy.sleep(0.1)
