cmake_minimum_required(VERSION 2.8.3)
project(katana_arm_gazebo)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  urdf
  xacro
  robot_state_publisher
  katana_description
  controller_manager
  katana_gazebo_plugins
  joint_trajectory_controller
  actionlib
  controller_manager_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  roscpp
  gazebo_ros
  urdf
  xacro
  robot_state_publisher
  katana_description
  controller_manager
  katana_gazebo_plugins
  joint_trajectory_controller
  actionlib
  controller_manager_msgs
  std_msgs
  DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(min_max_trajectory src/min_max_trajectory.cpp)
target_link_libraries(min_max_trajectory ${catkin_LIBRARIES})
add_dependencies(min_max_trajectory ${catkin_EXPORTED_TARGETS})

install(TARGETS min_max_trajectory
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
