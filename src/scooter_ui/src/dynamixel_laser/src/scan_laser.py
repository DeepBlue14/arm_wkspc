#!/usr/bin/python

import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
import curses

# Scan a laser over a grid of points for attemping grasps. 

class DoneException(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
         return repr(self.value)
     
servoStates = {}

def waitOnServos():
    #Don't call this without initializing the node first. 
    #Get the time for timeout
    startTime = rospy.Time.now()
    
    #First, wait for all servo updates to have occured AFTER startup
    servoIsStale = True
    while servoIsStale:
        servoIsStale = False
        for key in servoStates.keys():
                if servoStates[key].header.stamp < startTime:
                    #rospy.logwarn("Stale servo state detected, waiting... ({0} < {1})".format(servoStates[key].header.stamp, startTime))
                    servoIsStale = True
        rospy.sleep(0.001)
    #rospy.loginfo("Stale info wait ended after {0}".format(rospy.Time.now()-startTime))
    
    #While the servos are moving, don't leave this function
    #Get the time for timeout
    startTime = rospy.Time.now()
    anyMotion = True
    while anyMotion:
        if startTime + rospy.Duration(20.0) > rospy.Time.now():
            anyMotion = False
            for key in servoStates.keys():
                anyMotion |= servoStates[key].is_moving
            #If any of the servos were moving, anyMotion is now True
        else:
            #If it has been more than 20 seconds
            rospy.logwarn("Timed out waiting for motion to stop.")
            anyMotion = False
        rospy.sleep(0.001)
    #rospy.loginfo("Motion wait ended after {0}".format(rospy.Time.now()-startTime))
    
#Just stores the status of each joint as it comes in
def handleServoStatus(status):
    servoStates[status.name] = status
    

def floatRange(start, stop, step):
    current = start
    if start < stop:
        while current < stop:
            yield current
            current += step
    else:
        while current > stop:
            yield current
            current -= step
            

#Load the input file and get the scan values
maxPan = maxTilt = minPan = minTilt = 0
with open("./laser_ranges.csv", 'r') as infile:
    values = infile.read()
    
    maxPan, maxTilt, minPan, minTilt = [float(x) for x in values.split(',')]
    #If the minima are greater than the maxima, swap them so scans always go in the same direction 
    if maxPan < minPan:
        maxPan, minPan = minPan, maxPan
    if maxTilt < minTilt:
        maxTilt, minTilt = minTilt, maxTilt

# It's kind of hard to tell what the units are for the dynamixel motions, 
# since it's not in the documentation, but it seems like the range is 
# about +/-3, so I'd assume radians.
startPos = (maxPan, maxTilt)
endPos = (minPan, minTilt)

#Set up to talk to ROS
rospy.init_node('laser_pan_tilt', anonymous=True)
panPub = rospy.Publisher("/pan_controller/command", Float64, queue_size=10)
tiltPub = rospy.Publisher("/tilt_controller/command", Float64, queue_size=10)

panStateSub = rospy.Subscriber("/tilt_controller/state", JointState, handleServoStatus)
tiltStateSub = rospy.Subscriber("/pan_controller/state", JointState, handleServoStatus)

#Move to start position
tiltPub.publish(round(startPos[0], 5))
waitOnServos()
panPub.publish(round(startPos[1], 5))
waitOnServos()

#Set up curses
stdscr = curses.initscr()
curses.noecho()
curses.cbreak()
stdscr.keypad(1)

#Watch for and break loop
done = False
step = 0.01
try:
    #Add step to range so it's inclusive rather than exclusive
    for tilt in floatRange(startPos[1], endPos[1] + step, step):
        tiltPub.publish(round(tilt, 5))
        waitOnServos()
        for pan in floatRange(startPos[0], endPos[0] + step, step):
            panPub.publish(round(pan, 5))
            waitOnServos()
            #Block waiting for user IO
            key = stdscr.getch()
            if key == ord('q'):
                raise DoneException("Break out of nested loop")
except DoneException:
    pass #Breaking out of multiply nested loop
                    
#Done with main loop, clean up curses
curses.nocbreak(); stdscr.keypad(0); curses.echo()
curses.endwin()
        
        #Attempt grasp

        #Prompt user to indicate if grasp succeded or failed

        #Move to next position
