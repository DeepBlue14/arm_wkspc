roslaunch active_sensing active_sensing.launch
roslaunch agile_grasp2 fixed_detect_grasps_active.launch
rosrun baxter_tools enable_robot.py -e
roslaunch scooter_launch three_cam.launch

