#!/usr/bin/env python

########################################################################
##
## Test program
## 
##
#########################################################################

from random import randint

import rospy
from std_msgs.msg import Int32

def talker():
    options = ['stop', 'greet', 'init', 'cal0', 'cal1', 'cal2', 'cal3', 'cal4']
    pub = rospy.Publisher('chatter', Int32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        num = randint(0, len(options)-1)
        rospy.loginfo(num)
        pub.publish(num)
        rate.sleep()
        rospy.sleep(5)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

