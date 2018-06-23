#!/usr/bin/python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from dynamixel_msgs.msg import JointState

#Scans the dynamixels in response to a joystick

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
            

def updatePosition(jstkMsg):
    # It's kind of hard to tell what the units are for the dynamixel motions, 
    # since it's not in the documentation, but it seems like the range is 
    # about +/-3, so I'd assume radians.
    #forward/reverse on stick
    currTilt = 2 * jstkMsg.axes[1]
    #left/right on stick
    currPan = 2 * jstkMsg.axes[0]
    
    panPub.publish(round(currPan, 5))
    tiltPub.publish(round(currTilt, 5))
    #waitOnServos()
    
#Set up to talk to ROS
rospy.init_node('laser_pan_tilt', anonymous=True)
panPub = rospy.Publisher("/pan_controller/command", Float64, queue_size=10)
tiltPub = rospy.Publisher("/tilt_controller/command", Float64, queue_size=10)

panStateSub = rospy.Subscriber("/tilt_controller/state", JointState, handleServoStatus)
tiltStateSub = rospy.Subscriber("/pan_controller/state", JointState, handleServoStatus)

joystickSub = rospy.Subscriber("/joy", Joy, updatePosition)

#TODO collect dynamixel limits and add bounds checking
#Move to start position
panPub.publish(round(0, 5))
tiltPub.publish(round(0, 5))
waitOnServos()

rospy.spin()






