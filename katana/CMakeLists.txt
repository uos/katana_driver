cmake_minimum_required(VERSION 2.8.3)
project(katana)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  kni
  std_srvs
  sensor_msgs
  actionlib
  control_msgs
  trajectory_msgs
  urdf
  tf
  katana_msgs
  geometry_msgs
  moveit_msgs
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    roslib
    kni
    std_srvs
    sensor_msgs
    actionlib
    control_msgs
    trajectory_msgs
    urdf
    tf
    katana_msgs
    geometry_msgs
    moveit_msgs
    geometry_msgs
  DEPENDS
    Boost
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(katana src/KatanaNode.cpp
                      src/AbstractKatana.cpp
                      src/Katana.cpp
                      src/Katana300.cpp
                      src/SimulatedKatana.cpp
                      src/JointStatePublisher.cpp
                      src/joint_trajectory_action_controller.cpp
                      src/joint_movement_action_controller.cpp
                      src/KNIConverter.cpp
                      src/spline_functions.cpp
                      src/katana_gripper_grasp_controller.cpp)
target_link_libraries(katana ${catkin_LIBRARIES})
add_dependencies(katana ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(katana_arm_kinematics src/KNIConverter.cpp
                      src/KNIKinematics.cpp)
target_link_libraries(katana_arm_kinematics ${catkin_LIBRARIES})
add_dependencies(katana_arm_kinematics ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS katana katana_arm_kinematics
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS nodes/fake_katana_joint_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
