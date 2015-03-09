cmake_minimum_required(VERSION 2.8.3)
project(katana_msgs)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  actionlib_msgs
  message_generation
)

add_message_files(
  FILES
  GripperControllerState.msg
)

add_action_files(
  FILES
  JointMovement.action
)

generate_messages(
  DEPENDENCIES
  sensor_msgs actionlib_msgs)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    message_runtime
    sensor_msgs
    actionlib_msgs
  DEPENDS
)
