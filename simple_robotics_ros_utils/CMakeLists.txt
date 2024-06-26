cmake_minimum_required(VERSION 3.0.2)
project(simple_robotics_ros_utils)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++20)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_robotics_ros_utils
#  CATKIN_DEPENDS rosbag roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
  ${catkin_INCLUDE_DIRS}
)

