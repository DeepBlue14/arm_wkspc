#!/usr/bin/env python
'''Broadcasts the transform for the sensor.

Credits:
- http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20%28Python%29

Assembled: Northeastern University, 2015
'''

import rospy
import tf

if __name__ == '__main__':

  rospy.init_node('structure_tf_broadcaster')
  br = tf.TransformBroadcaster()
  rate = rospy.Rate(50.0)

  while not rospy.is_shutdown():
    br.sendTransform((-0.060, 0.036, -0.035), (0, -0.70710678118, 0, 0.70710678118), rospy.Time.now(), \
      "structure_link", "left_gripper") # structure measured
    rate.sleep()
    
