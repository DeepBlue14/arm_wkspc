cmake_minimum_required(VERSION 2.8.3)
project(qrcode_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  QRCode.msg
  QRCodes.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES calibrate_cams
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs message_runtime
#  DEPENDS system_lib
)