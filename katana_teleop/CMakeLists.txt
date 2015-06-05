cmake_minimum_required(VERSION 2.8.3)
project(katana_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  control_msgs
  sensor_msgs
  katana_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    actionlib
    control_msgs
    sensor_msgs
    katana_msgs
  DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(katana_teleop_key src/katana_teleop_key.cpp)
target_link_libraries(katana_teleop_key ${catkin_LIBRARIES})
add_dependencies(katana_teleop_key ${catkin_EXPORTED_TARGETS})

install(TARGETS katana_teleop_key
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
