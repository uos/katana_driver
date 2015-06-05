cmake_minimum_required(VERSION 2.8.3)
project(katana_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  trajectory_msgs
  control_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
  roscpp
  actionlib
  trajectory_msgs
  control_msgs
  sensor_msgs
  DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(follow_joint_trajectory_client src/follow_joint_trajectory_client.cpp)
target_link_libraries(follow_joint_trajectory_client ${catkin_LIBRARIES})
add_dependencies(follow_joint_trajectory_client ${catkin_EXPORTED_TARGETS})

add_executable(pr2_joint_trajectory_client src/pr2_joint_trajectory_client.cpp)
target_link_libraries(pr2_joint_trajectory_client ${catkin_LIBRARIES})
add_dependencies(pr2_joint_trajectory_client ${catkin_EXPORTED_TARGETS})

install(TARGETS follow_joint_trajectory_client pr2_joint_trajectory_client
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
