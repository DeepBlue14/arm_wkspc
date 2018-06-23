#! /usr/bin/env python

#####
##
## File Description: This class converts input from a mouse to ROS Joy-style
##                   messages.
##
#####

try:
    from sensor_msgs.msg import Joy
except:
    import roslib; roslib.load_manifest("mouse_joy_pos")
    from sensor_msgs.msg import Joy

class VirtualJoy():
    def __init__(self):
        joy_pub = rospy.Publisher('joy', Joy, queue_size=2)
    
    
    def mouse2joy(self, mouseX, mouseY, bLeft, bMiddle, bRight):
        #Fill out a joystick message and send it
        joy_msg = Joy()
        joy_msg.header.stamp = rospy.Time.now()
        joy_msg.header.frame_id = "mouse_joystick"
        joy_msg.axes = [mouseX, mouseY]
        joy_msg.buttons = [bLeft, bMiddle, bRight]
        #joy_msg.buttons = [0,0,0]
        joy_pub.publish(joy_msg)
        # Only update at a fixed rate
        r.sleep() 
