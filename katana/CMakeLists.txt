cmake_minimum_required(VERSION 2.8.3)
project(katana)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -l/home/hdeeken/ros/indigo/catkin_ws/src/katana_driver/kni/KNI_4.3.0/lib/linux/libKinematics.so" )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -l/home/hdeeken/ros/indigo/catkin_ws/src/katana_driver/kni/KNI_4.3.0/lib/linux/libKNIBase.so" )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -l/home/hdeeken/ros/indigo/catkin_ws/src/katana_driver/kni/KNI_4.3.0/lib/linux/libKNI_InvKin.so" )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -l/home/hdeeken/ros/indigo/catkin_ws/src/katana_driver/kni/KNI_4.3.0/lib/linux/libKNI_LM.so" )

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
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY nodes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
