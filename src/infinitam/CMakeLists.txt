cmake_minimum_required(VERSION 2.8.3)
project(infinitam)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation pcl_conversions roscpp sensor_msgs std_msgs)
  
## Find PCL libraries
find_package(PCL 1.8 EXACT REQUIRED)

## Specify message files
add_message_files(FILES volume.msg)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime DEPENDS PCL)

###########
## Build ##
###########

#set (INFINITAM_DIR /home/mgualti/Programs/InfiniTAM/InfiniTAM)
set(INFINITAM_DIR /home/james/Code/InfiniTAM/InfiniTAM)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${INFINITAM_DIR}
  /usr/local/cuda/include
)

add_executable(infinitam src/infinitam.cpp)

target_link_libraries(infinitam
  ${INFINITAM_DIR}/build/Engine/libEngine.a
  ${INFINITAM_DIR}/build/ITMLib/libITMLib.a
  ${INFINITAM_DIR}/build/Utils/libUtils.a
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  /usr/lib/libOpenNI2.so
  /usr/lib/x86_64-linux-gnu/libcuda.so.1
  /usr/local/cuda/lib64/libcudart.so
  /usr/local/cuda/lib64/libcublas.so
  /usr/local/cuda/lib64/libcufft.so
)

  
