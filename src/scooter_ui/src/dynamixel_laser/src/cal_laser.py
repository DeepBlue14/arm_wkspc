#!/usr/bin/python

import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
import curses

#Lets the user interactively set the start and end positions of a laser scan,
#For doing setup to scan the laser in a grid over an object. 

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
                    rospy.logwarn("Stale servo state detected, waiting...")
                    servoIsStale = True
                    rospy.sleep(0.1)
    
    #While the servos are moving, don't leave this function
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
        rospy.sleep(0.1)
    rospy.loginfo("Wait ended after {0}".format(rospy.Time.now()-startTime))
    
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
            
# It's kind of hard to tell what the units are for the dynamixel motions, 
# since it's not in the documentation, but it seems like the range is 
# about +/-3, so I'd assume radians.
#TODO Get start position
startPos = (3,3) #This is outside my max limits
#TODO Get end position
endPos = (-3, -3) #This is also outside my max limits

step = 0.01

#TODO Get distance to object? Alt: Use the same number of points for all objects, but closer together. 

#Set up to talk to ROS
rospy.init_node('laser_pan_tilt', anonymous=True)
panPub = rospy.Publisher("/pan_controller/command", Float64, queue_size=10)
tiltPub = rospy.Publisher("/tilt_controller/command", Float64, queue_size=10)

panStateSub = rospy.Subscriber("/tilt_controller/state", JointState, handleServoStatus)
tiltStateSub = rospy.Subscriber("/pan_controller/state", JointState, handleServoStatus)

#Move to start position
currPan = 0
currTilt = 0
panPub.publish(round(currPan, 5))
tiltPub.publish(round(currTilt, 5))
waitOnServos()

#Save the extents
maxInit = False
maxPan = 0
maxTilt = 0
minInit = False
minPan = 0
minTilt = 0

#Set up curses for immediate input and arrow keys
stdscr = curses.initscr()
curses.noecho()
curses.cbreak()
stdscr.keypad(1)

done = False

#Main loop, just get the user input and drive the servos
while not done:
    #Run the event pump to get characters
    c = stdscr.getch()
    moveNeeded = False
    #Up motion
    if c == curses.KEY_UP:
        currTilt += 0.01
        moveNeeded = True
    elif c == ord('w'):
        currTilt += 0.1
        moveNeeded = True
    #Down find and coarse
    elif c == curses.KEY_DOWN:
        currTilt -= 0.01
        moveNeeded = True
    elif c == ord('s'):
        currTilt -= 0.1
        moveNeeded = True
    #Left fine and coarse
    elif c == curses.KEY_LEFT:
        currPan += 0.01
        moveNeeded = True        
    elif c == ord('d'):
        currPan += 0.1
        moveNeeded = True
    #Right fine and coarse
    elif c == curses.KEY_RIGHT:
        currPan -= 0.01
        moveNeeded = True
    elif c == ord('a'):
        currPan -= 0.1
        moveNeeded = True
    #Save values
    elif c == ord(' '):
        if not maxInit:
            maxPan = currPan
            maxTilt = currTilt
            maxInit = True
        else:
            minPan = currPan
            minTilt = currTilt
            minInit = True
    else:
        done = True
    
    if moveNeeded:
        panPub.publish(round(currPan, 5))
        tiltPub.publish(round(currTilt, 5))
        waitOnServos()

#Done with main loop, clean up and dump config to file? screen?
curses.nocbreak(); stdscr.keypad(0); curses.echo()
curses.endwin()

#Print the values
if maxInit:
    if not minInit:
            minPan = currPan
            minTilt = currTilt
            minInit = True        
    print maxPan, maxTilt, minPan, minTilt
    with open("./laser_ranges.csv", 'w') as outfile:
        outfile.write("{0},{1},{2},{3}".format(maxPan, maxTilt, minPan, minTilt))
else:
    print "Did not get max values"