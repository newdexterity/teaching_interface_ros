cmake_minimum_required(VERSION 2.8.3)
project(teaching_interface_ros)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(SERVO_LIB_NAME moveit_servo_cpp_api)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  control_msgs
  tf2
  tf2_ros
  message_generation
  moveit_servo
)

add_message_files(
  FILES
  TeachingCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  control_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    message_runtime
    moveit_servo
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Servo node translating interface commands into robot motion
add_executable(servo_node src/servo_node.cpp)
add_dependencies(servo_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(servo_node
  ${catkin_LIBRARIES}
)