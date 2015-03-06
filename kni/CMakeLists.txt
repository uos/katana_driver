cmake_minimum_required(VERSION 2.8.3)
project(kni)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS KNI_4.3.0/include
  LIBRARIES KNIBase Kinematics KNI_InvKin KNI_LM
  CATKIN_DEPENDS
  DEPENDS Boost
)

include(ExternalProject)

externalproject_add(KNI_4.3.0_common
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/common/
  CONFIGURE_COMMAND ""
  BINARY_DIR ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/common/
  INSTALL_COMMAND ""
)

externalproject_add(KNI_4.3.0_Base
  DEPENDS KNI_4.3.0_common
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/Base/
  CONFIGURE_COMMAND ""
  BINARY_DIR ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/Base/
  INSTALL_COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/KNI_4.3.0/lib/linux/libKNIBase.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libKNIBase.so
)

externalproject_add(KNI_4.3.0_roboop
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/KNI_4.3.0/lib/kinematics/roboop/
  CONFIGURE_COMMAND ""
  BINARY_DIR ${PROJECT_SOURCE_DIR}/KNI_4.3.0/lib/kinematics/roboop/
  INSTALL_COMMAND ""
)

externalproject_add(KNI_4.3.0_kinematics
  DEPENDS KNI_4.3.0_roboop
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/KNI_4.3.0/lib/kinematics/
  CONFIGURE_COMMAND ""
  BINARY_DIR ${PROJECT_SOURCE_DIR}/KNI_4.3.0/lib/kinematics/
  INSTALL_COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/KNI_4.3.0/lib/linux/libKinematics.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libKinematics.so
)

externalproject_add(KNI_4.3.0_InvKin
  DEPENDS KNI_4.3.0_kinematics
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/InvKin/
  CONFIGURE_COMMAND ""
  BINARY_DIR ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/InvKin/
  INSTALL_COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/KNI_4.3.0/lib/linux/libKNI_InvKin.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libKNI_InvKin.so
)

externalproject_add(KNI_4.3.0_LM
  DEPENDS KNI_4.3.0_common KNI_4.3.0_Base
  SOURCE_DIR ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/LM/
  CONFIGURE_COMMAND ""
  BINARY_DIR ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/LM/
  INSTALL_COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/KNI_4.3.0/lib/linux/libKNI_LM.so ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libKNI_LM.so
)

add_library(KNIBase SHARED)
set_target_properties(KNIBase PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(KNIBase KNI_4.3.0_Base)
add_library(Kinematics SHARED)
set_target_properties(Kinematics PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(Kinematics KNI_4.3.0_kinematics)
add_library(KNI_InvKin SHARED)
set_target_properties(KNI_InvKin PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(KNI_InvKin KNI_4.3.0_InvKin)
add_library(KNI_LM SHARED)
set_target_properties(KNI_LM PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(KNI_LM KNI_4.3.0_LM)

install(TARGETS KNIBase Kinematics KNI_InvKin KNI_LM
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY KNI_4.3.0/include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(DIRECTORY KNI_4.3.0/configfiles300
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/KNI_4.3.0/)

install(DIRECTORY KNI_4.3.0/configfiles400
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/KNI_4.3.0/)

install(DIRECTORY KNI_4.3.0/configfiles450
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/KNI_4.3.0/)
