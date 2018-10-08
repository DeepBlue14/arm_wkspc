# active_sensing

* **Authors:** Marcus Gualteri, Andreas ten Pas, Abe Shultz, James Kuczynski
* **Version:** 1.1.0


# ActiveSensingDriving =====================================

Remember to start baxter.sh in ros_indigo_ws for each terminal.

1. Start avahi:

  sudo avahi-autoipd eth0

1.5 Fix the NTP clock if needed. 

  On the laptop (sets it from the real world):
  sudo service ntp stop
  sudo ntpdate 0.north-america.pool.ntp.org
  sudo service ntp start

  On the backpack PC (sets it from the laptop):
  sudo service ntpd stop
  sudo ntpdate 169.254.196.178
  sudo service ntpd start

2. Start rviz:
   
  roslaunch active_sensing active_sensing.launch
  rqt
   
3. Launch the point cloud for the base sensor:

  roslaunch scooter_launch three_cam.launch
  *OR*
  roslaunch scooter_launch two_cam.launch

5. Launch the agile_grasp grasp pose detection:

  roslaunch agile_grasp2 fixed_detect_grasps_active.launch

6. Start laser point detection:

  roslaunch laser_detection_3d 3dlaser.launch

7. Turn the robot on:

   rosrun baxter_tools enable_robot.py -e

8. Run automatic OR manual scenario:
   
  python src/active_sensing/scripts/active_sensing_driving.py 1 0
  python src/active_sensing/scripts/user_keyboard_control.py

# Bugs =====================================

 1. "robot dof values don't match initialization"
   Any initial trajectory to trajopt that is length 1 will cause this error.

