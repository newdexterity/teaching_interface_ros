cmake_minimum_required(VERSION 2.8.3)
project(teaching_interface_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  tf2
  tf2_ros
  message_generation
)

add_message_files(
  FILES
  InterfaceData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
