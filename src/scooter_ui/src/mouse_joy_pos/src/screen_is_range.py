#!/usr/bin/python

#A completely different way of getting the mouse pointer to act as a joystick. 
#In this case, instead of accumulating mouse deltas, we just convert the mouse's position on the 
#screen into a value in the range (-1,1) with -1 at the left and bottom, and 1 at the top and right. 
#Yes, this means that the resolution of your joystick now depends on the resolution of your monitor. 
#Don't like it? USE A JOYSTICK FOR YOUR JOYSTICK.

 
import rospy
import struct
import select
import os
import pyudev
import sys

try:
    from sensor_msgs.msg import Joy
except:
    import roslib; roslib.load_manifest("mouse_joy_pos")
    from sensor_msgs.msg import Joy

try:
    import Xlib
    import Xlib.display
    import select
except:
    rospy.logerror("This needs python-xlib, which can be installed with apt on Ubuntu.")
    sys.exit(-1)
         
def main(device_name, width, height):
    joy_pub = rospy.Publisher('joy', Joy, queue_size=2)
    rootDisp = Xlib.display.Display().screen().root
    with open(device_name, "rb" ) as tp_file:
        #Set the rate of the mouse check loop, in Hz
        r = rospy.Rate(50);
        while not rospy.is_shutdown():
            #Get the position of the mouse with the origin at the center of the screen
            data = rootDisp.query_pointer()._data
            mouseX = (data["root_x"]-(width/2))/(float(width)) * 2 
            mouseY = (data["root_y"]-(height/2))/(float(height)) * 2
            
            # Wait for display to send something, or a timeout of one second
            #readable, w, e = select.select([rootDisp.display], [], [], 1)

            # if no files are ready to be read, it's an timeout
            #if readable:
                # if display is readable, handle as many events as have been recieved
            #    rospy.logInfo(readable)
            #    if rootDisp.display in readable:
            #        i = rootDisp.display.pending_events()
            #        while i > 0:
            #            event = rootDisp.display.next_event()
            #            rospy.loginfo(event)
            #            i = i - 1
                
            #rospy.loginfo("({0},{1})".format(mouseX, mouseY))
            
            #Get the mouse buttons 
            buf = tp_file.read(3)
            button = ord( buf[0] )
            bLeft = ( button & 0x1 ) > 0
            bMiddle = ( button & 0x4 ) > 0
            bRight = ( button & 0x2 ) > 0
            
            #rospy.loginfo("({0},{1}) {2} {3} {4}".format(mouseX, mouseY, bLeft, bMiddle, bRight))
            
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
    
if __name__ == "__main__":
    rospy.init_node("mouse_joy_pos")
    #If the device name was specified, find the path, otherwise if the path was specified, use that
    #Defaults to the 0th mouse device, which is probably the one you use for mousing around your screen
    device_name = rospy.get_param("~dev_name", False)
    if device_name:
        dev = lookupDeviceFileByNameAttribute(device_name)
    else:
        dev = rospy.get_param("~dev", "/dev/input/mice")
    #Get the screen width and height 
    display = Xlib.display.Display()
    root = display.screen().root
    desktop = root.get_geometry()
    #Only care about mouse 
    root.change_attributes(event_mask= Xlib.X.ButtonPressMask | Xlib.X.ButtonReleaseMask)
    rospy.loginfo("Starting with {0} in a {1}x{2} desktop".format(dev, desktop.width, desktop.height))
    main(dev, desktop.width, desktop.height)
    
