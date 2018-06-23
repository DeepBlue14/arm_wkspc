#!/usr/bin/python

import curses
import atexit
import time
import rospy
import copy
import baxter_interface
from baxter_core_msgs.msg import CollisionDetectionState
from agile_grasp.msg import Grasps
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from grasp_selection.srv import *

from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)

import numpy as np

#Clean up curses UI stuff
def cleanup():
    curses.nocbreak()
    curses.echo()
    curses.endwin()

#Nonblocking motion without threads, wooo!
#If you don't call update() often enough, the motion gets jerky
class Mover():
    def __init__(self):
        self.angles = None
        self.arm = baxter_interface.Limb("left")
        self.hand = baxter_interface.Gripper('left', baxter_interface.CHECK_VERSION)
        self.inCollision = False
        self.threshold = 0.01 #For checking positions
        self.waypointIndex = 0 #Start with the first waypoint for waypoint moves
        self.inWaypointMove = False
        self.waypoints = None
        
    def update(self):
        if self.angles is None:
            #No angles yet
            return
        if self.inWaypointMove:
            #Check if we've finished moving to the last waypoint
            if self.waypointIndex >= len(self.waypoints) and self.doneMoving():
                status.writeStatus("Done moving")
                self.inWaypointMove = False
                self.waypoints = None
            else:
                #If we're done moving, start moving to the next waypoint, if there is one
                if self.doneMoving():
                    status.writeStatus("Next waypoint")
                    #On to the next waypoint, if there is one
                    self.waypointIndex += 1
                    if self.waypointIndex >= len(self.waypoints):
                        #This is past the last waypoint
                        status.writeStatus("Last waypoint")
                        self.inWaypointMove = False
                        self.waypoints = None
                        return
                    else:
                        self.moveIK(self.waypoints[self.waypointIndex])
                    
                    
                        
        self.arm.set_joint_positions(self.angles)
        
    def moveTo(self, angles, speed = 0.3):
        self.arm.set_joint_position_speed(speed)
        self.angles = angles
    
    #Note THIS BLOCKS, and will make the UI non-responsive
    def moveToBlocking(self, angles, speed = 0.3):
        self.arm.set_joint_position_speed(speed)
        self.angles = angles()
        self.arm.move_to_joint_positions(self.angles)

    def update_collision(self, msg):
        self.inCollision = msg.collision_state
            
    def stop(self):
        #Next update will make arm move to the position that it's already in
        self.angles = self.arm.joint_angles()
        self.update()
        
    def closeHand(self):
        self.hand.close(block=True)
    
    def openHand(self):
        self.hand.open(block=True)
    
    def doneMoving(self):
        #Only done if we're not doing a waypoint move and at the right location
        return self.inPosition(self.angles)
        
    def inPosition(self, joint_position):
        #Not commanded to any position, so wherever we are is fine
        if joint_position is None:
            return true
        #Check that the joint angles are within a threshold of the desired angles
        joint_vals = np.array(self.arm.joint_angles().values())
        query_position = np.array(joint_position.values())
        joint_error = np.linalg.norm(query_position-joint_vals)
        if joint_error < self.threshold:
            return True
        else:
            return False
                              
    def hasCollided(self):
        return self.inCollision
    
    def moveWaypoint(self, wpList):
        #Not currently in a waypoint move, so start a new one
        if not self.inWaypointMove:
            self.waypointIndex = 0
            self.waypoints = wpList
            self.inWaypointMove = True

                    
    #Use the IK solver to move to a waypoint
    #Returns true if IK could find a solution, false if not
    def moveIK(self, waypoint, speed = 0.3):
        ik_good = True
        response = self.callBuiltInIK(waypoint.position, waypoint.orientation)
        if response.isValid:
            solution = dict(zip(response.joints[0].name, response.joints[0].position))
            self.moveTo(solution, speed)
            if status is not None:
                status.writeStatus("Updated mover to waypoint")
        else:
            # No valid solutions for this waypoint, fail
            if status is not None:
                status.writeStatus("IK solver didn't return any valid solutions for waypoint.")
            ik_good = False
        return ik_good
    
    def callBuiltInIK(self, pos, rot):
        status.writeStatus("called callBuiltInIK")
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
            #self.statu print "IK Service call failed %s" % e
            #TODO handle this error, although it almost never happens
            pass

#Handles displaying strings in the status area
class StatusArea():
    def __init__(self, screen, len = 10, loc = (7,1), writeLog = False):
        self.messages = []
        self.scr = screen
        self.location = loc
        self.len = len
        if writeLog:
            self.logfile = open("./status_log.txt", 'w')
            self.writeLog = True
    
    def writeStatus(self, message):
        #Add a time to the message and stick it in the buffer
        prefix = time.strftime("[%X] ") 
        
        #Optional writing to a log file
        if self.writeLog and self.logfile is not None:
            self.logfile.write(prefix + message + "\n")
            
        self.messages.append(prefix + message)
        if len(self.messages) > self.len:
            self.messages = self.messages[1:]
        for index in range(len(self.messages)):
            #Delete it, clear to eol, add new line
            self.scr.move(self.location[0] + index, self.location[1])
            self.scr.clrtoeol()
            self.scr.addstr(self.location[0] + index, self.location[1], self.messages[index])
        self.scr.refresh() 


#State machine implementation turned out to be a bad move
#This class handles the motions and keeps track of the state of the robot
class RobotStateModel():
    def __init__(self, mover, status):
        self.mover = mover
        self.statusArea = status
        self.mid_position = {"left_s0" :0.162218468134,
                             "left_s1" :-1.68776235994,
                             "left_e0" :0.589815612268,
                             "left_e1" :2.21698573116,
                             "left_w0" :3.05914118275,
                             "left_w1" :-0.999388482166,
                             "left_w2" :0.743980681274}

        self.home_position = {"left_s0" :-0.0579077746765,
                              "left_s1" :-2.14680611019,
                              "left_e0" :-0.0322135965088,
                              "left_e1" :1.93511676171,
                              "left_w0" :-0.00421844716187,
                              "left_w1" :1.76676237043,
                              "left_w2" :-0.0011504855896}
        self.grasp = None
        self.limb = baxter_interface.Limb('left')
        self.haveGrasp = False
        self.attemptStarted = False
        self.waypointMoveStarted = False
        
    def update(self):
        self.mover.update()
        if self.attemptStarted and self.haveGrasp:
            if not self.waypointMoveStarted:
                self.statusArea.writeStatus("Getting waypoints")
                #Get the positions for the grasp
                wpl, speeds = self.getWayPoints()
                #This does very little if we're already in a waypoint move
                self.mover.moveWaypoint(wpl)
                self.waypointMoveStarted = True
            elif not self.mover.inWaypointMove:
                #The attempt has started, and the waypoint move started, and now the move is over
                self.statusArea.writeStatus("inWaypointMove {0}, doneMoving {1}, attemptStarted{2})".format(self.mover.inWaypointMove, self.mover.doneMoving(), self.attemptStarted))
                self.mover.closeHand()
                self.regrasp()
                self.lift()
                self.moveHome()
                self.attemptStarted = False
                self.waypointMoveStarted = False
        elif self.mover.inPosition(self.home_position):
             #Only get grasps if robot is in the home position
            self.getGrasps()
        
    
    def getGrasps(self):
        try:
            # get the current pose of the robot
            pose = self.limb.endpoint_pose() 
            # convert to a geometry_msg
            pose = Pose(pose['position'], pose['orientation'])
            # Get the poses
            response = select_grasps(pose)
            # If no exception was thrown, we got a proposed grasp
            if len(response.grasps.grasps) > 0:
                self.grasp = response.grasps.grasps[0]
                self.haveGrasp = True
                #TODO update visualization
                self.statusArea.writeStatus("Got a grasp")
                
        except rospy.service.ServiceException as e:
            self.statusArea.writeStatus(e.message)
    
    #Really just clears the grasp out so we don't use a (potentially stale) grasp
    def startAttempt(self):
        self.mover.openHand()
        self.statusArea.writeStatus("Starting attempt")
        self.grasp = None
        self.haveGrasp = False
        self.attemptStarted = True
        
    def dropObject(self):
        self.mover.openHand()
        
    #From a grasp, get the waypoints needed to move the arm
    def getWayPoints(self):
        # The waypoints allow the arm to approach the object slowly along the chosen approach axis
        num_waypoints = 3
        dist = [0.12, 0.05, 0.0]
        speeds = [0.3, 0.2, 0.08]
        threshs = [0.08, 0.03, 0.03]
        waypoints = [None] * num_waypoints
        for i in range(0, num_waypoints):
            wpose = copycd .deepcopy(self.grasp.pose)
            wpose.position.x = wpose.position.x - dist[i] * self.grasp.approach.x
            wpose.position.y = wpose.position.y - dist[i] * self.grasp.approach.y
            wpose.position.z = wpose.position.z - dist[i] * self.grasp.approach.z
            waypoints[i] = copy.deepcopy(wpose)    
            self.statusArea.writeStatus("---- waypoint " + str(i) + "----")
            self.statusArea.writeStatus(str(waypoints[i].position))
        return waypoints, speeds

    def moveHome(self):
        self.mover.moveTo(self.home_position, speed = 0.1)
        
    def moveCenter(self):
        self.mover.moveTo(self.mid_position)

    def regrasp(self, speed = 0.08):
        #Create a new waypoint 2cm away from previous
        wp_up = copy.deepcopy(self.grasp.pose)
        wp_up.position.x = wp_up.position.x - 0.02 * self.grasp.approach.x
        wp_up.position.y = wp_up.position.y - 0.02 * self.grasp.approach.y
        wp_up.position.z = wp_up.position.z - 0.02 * self.grasp.approach.z
        #Attempt to move to it, then reclose the gripper
        if self.mover.moveIK(wp_up, speed = speed):
            #This will cause a slight lag in the UI, but it's a small motion
            while not self.mover.doneMoving():
                self.mover.update()
            self.mover.closeHand()
        else:
            #TODO handle error
            pass
        
    def lift(self, speed = 0.08):
        #Start the lift from the grasp pose
        wp_lift = copy.deepcopy(self.grasp.pose)
        #Move 10cm straight up
        wp_lift.position.z = wp_lift.position.z + 0.1
        if self.mover.moveIK(wp_lift, speed = speed):
            #This will cause a slight lag in the UI, but it's a small motion
            while not self.mover.doneMoving():
                self.mover.update()    
        else:
            #TODO Handle IK error
            pass
    
    def stopMotion(self):
        self.mover.stop()
        
    def hasCollided(self):
        return self.mover.hasCollided()
    
class statTracker():
    def __init__(self):
        self.attempts = 0
        self.successes = 0
        self.isAttempting = False
        
    def noteStart(self):
        self.isAttempting = True
        self.attempts += 1
    
    def noteEnd(self, success = False):
        if isAttempting:
            self.isAttempting = False
            if success:
                self.successes += 1
    
    def quit(self):
        #TODO write out to a log file or something
        pass

if __name__ == '__main__':
    #Cleanup runs when we leave the script
    atexit.register(cleanup)
    
    #Init a curses screen
    stdscr = curses.initscr()
    #Don't echo characters
    curses.noecho()
    #Get keys instantly, rather than waiting for enter
    curses.cbreak()
    #Get single codes for keypads, rather than control sequences
    stdscr.keypad(1)
    #Make getch() non-blocking, returns curses.ERR if no input
    stdscr.nodelay(1)
    
    #Set up ROS parts
    rospy.init_node("single_cam_demo")

    ##For publishing markers so rviz can see what is going on
    #target_pub = rospy.Publisher('target', Marker, queue_size=1)
    #wayViz = WaypointViz()
    
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
            left_hand.calibrate()
    # open robot hand
    left_hand.open(block=True)
    
    #Mover takes care of doing motions
    mover = Mover()
    
    #Subscribe to collision detection, to stop if collision is detected
    collisions = rospy.Subscriber("/robot/limb/left/collision_detection_state", CollisionDetectionState, mover.update_collision)
        
    #Track attempts to grab the arm and successful grasps
    logStats = statTracker()
    
    #Print some help text
    stdscr.addstr(1, 1, "Commands", curses.A_REVERSE)
    stdscr.addstr(2, 1, "q - quit                a - attempt grasp")
    stdscr.addstr(3, 1, "c - center position     d - drop object")
    stdscr.addstr(4, 1, "h - home position       g - note that grasp was good/successful")
    stdscr.addstr(5, 1, "s - stop                b - note that grasp was bad/unsuccessful")
    stdscr.addstr(6, 1, "Status", curses.A_REVERSE)
    status = StatusArea(stdscr, len = 30, writeLog=True)
    
    stdscr.refresh()
    
    isRunning = True
    
    # Use the service that provides grasps that are easy to get to
    status.writeStatus("Waiting for /easy_grasps service")
    rospy.wait_for_service('/easy_grasps')
    select_grasps = rospy.ServiceProxy('/easy_grasps', SelectGrasps)
    status.writeStatus("Service found")
    
    #Set up the robot state object
    robotState = RobotStateModel( mover, status)
    
    #Move to the home position
    robotState.moveHome()
        
    #Main loop
    #The grasp attempt happens in stages
    # 1. User sets up object
    # 2. Robot attempts to find a grasp
    # 3. When a grasp is found, robot asks user to confirm
    # 4. Robot attempts to grab things
    # 5. Robot returns to home position
    # 6. User confirms or denies that the grasp was successful
    # 7. Robot releases the object
    while isRunning and not rospy.is_shutdown():
        c = stdscr.getch()
        if c != curses.ERR:
            #Got an input character
            if c == ord('h'):
                #This is deliberately throttled to avoid shaking the robot and 
                #breaking the calibration. 
                robotState.moveHome()
            elif c == ord('c'):
                robotState.moveCenter()
            elif c == ord('q'):
                isRunning = False
            elif c == ord('s'):
                robotState.stopMotion()
            elif c == ord('a'):
                logStats.noteStart()
                robotState.startAttempt()
            elif c == ord('d'):
                robotState.dropObject()
            elif c == ord('g'):
                logStats.noteEnd(success = True)
            elif c == ord('b'):
                logStats.noteEnd(success = False)
            else:
                print c

                        
        #Update the state model, which drives motion
        robotState.update()
        
        #Check if we ran into anything and stop
        if robotState.hasCollided():
            status.writeStatus("BOOM")
            robotState.stopMotion()
