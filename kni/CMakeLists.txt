cmake_minimum_required(VERSION 2.8.3)
project(kni)

find_package(catkin REQUIRED COMPONENTS )

catkin_package(
  INCLUDE_DIRS KNI_4.3.0/include
  LIBRARIES
  CATKIN_DEPENDS
)

execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/common/ make
                RESULT_VARIABLE _make_failed)
if(_make_failed)
  message(FATAL_ERROR "Build of common files failed")
endif(_make_failed)

execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/Base/ make
                RESULT_VARIABLE _make_failed)
if(_make_failed)
  message(FATAL_ERROR "Build of Base library failed")
endif(_make_failed)

execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/KNI_4.3.0/lib/kinematics/roboop/ make
                RESULT_VARIABLE _make_failed)
if(_make_failed)
  message(FATAL_ERROR "Build of roboop library failed")
endif(_make_failed)

execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/KNI_4.3.0/lib/kinematics/ make
                RESULT_VARIABLE _make_failed)
if(_make_failed)
  message(FATAL_ERROR "Build of kinematics library failed")
endif(_make_failed)

execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/InvKin/ make
                RESULT_VARIABLE _make_failed)
if(_make_failed)
  message(FATAL_ERROR "Build of InvKin library failed")
endif(_make_failed)

execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR}/KNI_4.3.0/src/LM/ make
                RESULT_VARIABLE _make_failed)
if(_make_failed)
  message(FATAL_ERROR "Build of LM library failed")
endif(_make_failed)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
