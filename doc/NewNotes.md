roslaunch active_sensing active_sensing.launch
roslaunch agile_grasp2 fixed_detect_grasps_active.launch
rosrun baxter_tools enable_robot.py -e
roslaunch scooter_launch three_cam.launch



=====
roslaunch active_sensing active_sensing.launch
roslaunch scooter_launch three_cam.launch
roslaunch agile_grasp2 fixed_detect_grasps_active.launch
roslaunch laer_detection_3d 3dlaser.launch
rosrun baxter_tools enable_robot.py -e
python src/active_sensing/scripts/active_sensing_driver.py 1 0
#rosrun cloud_registration cloud_registration


