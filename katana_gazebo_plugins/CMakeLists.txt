cmake_minimum_required(VERSION 2.8.3)
project(katana_gazebo_plugins)

#add_definitions(-fPIC)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  gazebo_ros
  actionlib
  trajectory_msgs
  control_msgs
  sensor_msgs
  katana_msgs
  trajectory_msgs
)

find_package(Boost REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_ros_katana_gripper
  CATKIN_DEPENDS
  gazebo_ros
  actionlib
  trajectory_msgs
  control_msgs
  sensor_msgs
  katana_msgs
  trajectory_msgs
  DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(gazebo_ros_katana_gripper src/gazebo_ros_katana_gripper.cpp src/katana_gripper_grasp_controller.cpp src/katana_gripper_joint_trajectory_controller.cpp src/gazebo_ros_katana_gripper_action_interface.cpp)
target_link_libraries(gazebo_ros_katana_gripper ${catkin_LIBRARIES})
add_dependencies(gazebo_ros_katana_gripper ${catkin_EXPORTED_TARGETS})

add_executable(test_gripper_joint_trajectory src/test_gripper_joint_trajectory.cpp)
target_link_libraries(test_gripper_joint_trajectory ${catkin_LIBRARIES})
add_dependencies(test_gripper_joint_trajectory ${catkin_EXPORTED_TARGETS})

install(TARGETS test_gripper_joint_trajectory
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS gazebo_ros_katana_gripper
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

####

#include_directories(${GAZEBO_INCLUDE_DIRS})
#link_directories(${GAZEBO_LIBRARY_DIRS})

#rosbuild_add_library(gazebo_ros_katana_gripper src/gazebo_ros_katana_gripper.cpp src/katana_gripper_grasp_controller.cpp src/katana_gripper_joint_trajectory_controller.cpp src/gazebo_ros_katana_gripper_action_interface.cpp)
#target_link_libraries(gazebo_ros_katana_gripper ${GAZEBO_LIBRARIES})

#rosbuild_add_executable(test_gripper_joint_trajectory src/test_gripper_joint_trajectory.cpp)
#target_link_libraries(test_gripper_joint_trajectory ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
