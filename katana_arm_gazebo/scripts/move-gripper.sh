#!/bin/sh

# close gripper
# GRASP=2
rostopic pub -1 /gripper_grasp_posture_controller/goal \
  object_manipulation_msgs/GraspHandPostureExecutionActionGoal -- \
  '{header: auto, goal: {grasp: {grasp_posture: {position: [0.0]}}, goal: 2}}'

# open gripper
# RELEASE=3
rostopic pub -1 /gripper_grasp_posture_controller/goal \
  object_manipulation_msgs/GraspHandPostureExecutionActionGoal -- \
  '{header: auto, goal: {goal: 3}}'

# move gripper to angle 0.0
# PRE_GRASP=1
rostopic pub -1 /gripper_grasp_posture_controller/goal \
  object_manipulation_msgs/GraspHandPostureExecutionActionGoal -- \
  '{header: auto, goal: {grasp: {grasp_posture: {position: [0.0]}, pre_grasp_posture: {position: [0.0]}}, goal: 1}}'
