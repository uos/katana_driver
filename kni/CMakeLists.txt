cmake_minimum_required(VERSION 2.8.3)
project(kni)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS KNI_4.3.0/include
  LIBRARIES KNIBase Kinematics KNI_InvKin KNI_LM
  CATKIN_DEPENDS
  DEPENDS Boost
)

include_directories(KNI_4.3.0/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# knicommon
aux_source_directory(KNI_4.3.0/src/common/ knicommon_SOURCES)
add_library(knicommon STATIC ${knicommon_SOURCES})

# KNIBase
aux_source_directory(KNI_4.3.0/src/Base/ KNIBase_SOURCES)
add_library(KNIBase SHARED ${KNIBase_SOURCES})
target_link_libraries(KNIBase knicommon)

# roboop
aux_source_directory(KNI_4.3.0/lib/kinematics/roboop/source/ roboop_SOURCES)
aux_source_directory(KNI_4.3.0/lib/kinematics/roboop/newmat/ roboop_SOURCES)
include_directories(include KNI_4.3.0/lib/kinematics/roboop/newmat/)
add_library(roboop STATIC ${roboop_SOURCES})

# Kinematics
aux_source_directory(KNI_4.3.0/lib/kinematics/ Kinematics_SOURCES)
aux_source_directory(KNI_4.3.0/lib/kinematics/AnalyticalGuess/src/ Kinematics_SOURCES)
include_directories(include KNI_4.3.0/lib/kinematics/AnalyticalGuess/include/)
add_library(Kinematics SHARED ${Kinematics_SOURCES})
target_link_libraries(Kinematics roboop)

# KNI_InvKin
aux_source_directory(KNI_4.3.0/src/InvKin/ KNI_InvKin_SOURCES)
add_library(KNI_InvKin SHARED ${KNI_InvKin_SOURCES})
target_link_libraries(KNI_InvKin Kinematics)

# KNI_LM
aux_source_directory(KNI_4.3.0/src/LM/ KNI_LM_SOURCES)
add_library(KNI_LM SHARED ${KNI_LM_SOURCES})
target_link_libraries(KNI_LM knicommon KNIBase)

set_target_properties(knicommon KNIBase roboop Kinematics KNI_InvKin KNI_LM 
  PROPERTIES COMPILE_FLAGS "-fPIC")

install(TARGETS KNIBase Kinematics KNI_InvKin KNI_LM
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY KNI_4.3.0/include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(DIRECTORY KNI_4.3.0/configfiles300
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/KNI_4.3.0/)

install(DIRECTORY KNI_4.3.0/configfiles400
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/KNI_4.3.0/)

install(DIRECTORY KNI_4.3.0/configfiles450
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/KNI_4.3.0/)
