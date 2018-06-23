import openravepy


# initialize environment
env = openravepy.Environment()
env.StopSimulation()
openrave_root = './'
env.Load(openrave_root + "baxter_ik.xml")
env.Lock()
env.SetViewer('qtcoin') # start the viewer (conflicts with matplotlib)
    
robot = env.GetRobots()[0] # get the first robot
manip = robot.SetActiveManipulator('left_arm') # set the manipulator to right_arm
s = raw_input("")

# set robot to dummy joint positions
dummy_joints = [0.017640779040527344, -1.074170045489502, 0.39269908125, 2.052466291845703, -0.4233786969726563, 0.6879903825805664, 0.031446606115722656]
robot.SetDOFValues(dummy_joints, manip.GetArmIndices()) # set the current solution
env.UpdatePublishedBodies() # allow viewer to update new robot

# read end-effector pose
current_pose = manip.GetEndEffectorTransform()
print current_pose

# try IK for similar pose
goal_pose = current_pose
goal_pose[2,3] -= 0.05
sol = manip.FindIKSolution(goal_pose, 18) # get collision-free solution
print "IK solution:", sol
s = raw_input("")
