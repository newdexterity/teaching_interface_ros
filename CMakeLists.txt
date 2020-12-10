cmake_minimum_required(VERSION 2.8.3)
project(teaching_interface_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  tf2
  tf2_ros
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
