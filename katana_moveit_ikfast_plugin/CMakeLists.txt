cmake_minimum_required(VERSION 2.8.3)
project(katana_moveit_ikfast_plugin)

# This works on CMake 2.8.11.2 (Ubuntu Saucy) and earlier, where other variants fail:
# - `add_compile_options(-std=c++11)` requires CMake >= 2.8.12 (Ubuntu Trusty)
# - `set_property(TARGET tgt PROPERTY CXX_STANDARD 11)` requires CMake >= 3.1.0
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(WARNING "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
)

include_directories(include)

set(IKFAST_LIBRARY_NAME katana_moveit_ikfast_kinematics_plugin)

find_package(LAPACK REQUIRED)

add_library(${IKFAST_LIBRARY_NAME} src/katana_450_6m90a_ikfast_plugin.cpp)
target_link_libraries(${IKFAST_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

install(TARGETS ${IKFAST_LIBRARY_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES
  katana_moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
