/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Author(s): Matei Ciocarlie

#include "object_manipulator/grasp_execution/grasp_executor_with_approach.h"

#include "object_manipulator/tools/hand_description.h"
#include "object_manipulator/tools/vector_tools.h"

//#include <demo_synchronizer/synchronizer_client.h>

using object_manipulation_msgs::GraspResult;
using motion_planning_msgs::ArmNavigationErrorCodes;

namespace object_manipulator {

/*! Disable collisions between end-effector and target */
motion_planning_msgs::OrderedCollisionOperations
GraspExecutorWithApproach::collisionOperationsForGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal)
{
  motion_planning_msgs::OrderedCollisionOperations ord;
  motion_planning_msgs::CollisionOperation coll;
  coll.object1 = handDescription().gripperCollisionName(pickup_goal.arm_name);
  coll.object2 = pickup_goal.collision_object_name;
  coll.operation = motion_planning_msgs::CollisionOperation::DISABLE;
  ord.collision_operations.push_back(coll);
  if (pickup_goal.allow_gripper_support_collision)
  {
    coll.object2 = pickup_goal.collision_support_surface_name;
    ord.collision_operations.push_back(coll);
  }
  /*
  // not sure we need this anymore
  ROS_DEBUG("Disabling collisions between fingertips and table for grasp");
  for (size_t i=0; i<handDescription().fingertipLinks(pickup_goal.arm_name).size(); i++)
  {
    coll.object1 = pickup_goal.collision_support_surface_name;
    coll.object2 = handDescription().fingertipLinks(pickup_goal.arm_name).at(i);
    ord.collision_operations.push_back(coll);
  }
  */
  ord.collision_operations = concat(ord.collision_operations,
                                    pickup_goal.additional_collision_operations.collision_operations);
  return ord;
}

/*! Zero padding on fingertip links */
std::vector<motion_planning_msgs::LinkPadding>
GraspExecutorWithApproach::linkPaddingForGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal)
{
	ROS_INFO("Setting FingertipPadding to zero");
	return concat(MechanismInterface::fingertipPadding(pickup_goal.arm_name, 0.0),
                pickup_goal.additional_link_padding);
}

GraspResult GraspExecutorWithApproach::getInterpolatedIKForGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
						     const object_manipulation_msgs::Grasp &grasp,
						     trajectory_msgs::JointTrajectory &grasp_trajectory)
{
 ROS_INFO("Start getIIK4Grasp..");
  //get the grasp pose in the right frame
  geometry_msgs::PoseStamped grasp_pose_stamped;
  grasp_pose_stamped.pose = grasp.grasp_pose;
  grasp_pose_stamped.header.frame_id = pickup_goal.target.reference_frame_id;
  grasp_pose_stamped.header.stamp = ros::Time(0);

  ROS_INFO("Executor_getIIK4Grasp: Pose %f %f %f in Frame: %s",  grasp_pose_stamped.pose.position.x,  grasp_pose_stamped.pose.position.y,  grasp_pose_stamped.pose.position.z,pickup_goal.target.reference_frame_id.c_str() );

  //use the opposite of the approach direction as we are going backwards, from grasp to pre-grasp
  geometry_msgs::Vector3Stamped direction;
  direction.header.stamp = ros::Time::now();
  direction.header.frame_id = handDescription().gripperFrame(pickup_goal.arm_name);
  direction.vector = mechInterface().negate( handDescription().approachDirection(pickup_goal.arm_name) );

  ROS_INFO("Executor_getIIK4Grasp: DirectionVector %f %f %f",  direction.vector.x,  direction.vector.y,  direction.vector.z);
  ROS_INFO("Executor_getIIK4Grasp: Distance %f",  pickup_goal.desired_approach_distance);

  std::vector<double> empty;
  //remember to pass that we want to flip the trajectory
  float actual_approach_distance;
  int error_code = mechInterface().getInterpolatedIK(pickup_goal.arm_name,
						     grasp_pose_stamped,
						     direction, pickup_goal.desired_approach_distance,
						     empty,
						     grasp.pre_grasp_posture,
						     collisionOperationsForGrasp(pickup_goal),
						     linkPaddingForGrasp(pickup_goal),
						     true, grasp_trajectory, actual_approach_distance);
  ROS_DEBUG("  Grasp executor approach distance: actual (%f), min(%f) and desired (%f)",
	   actual_approach_distance, pickup_goal.min_approach_distance, pickup_goal.desired_approach_distance);

  if ( actual_approach_distance < pickup_goal.min_approach_distance)
  {
    ROS_DEBUG("  Grasp executor: interpolated IK for grasp below min threshold");
    if (grasp_trajectory.points.empty())
    {
      ROS_DEBUG("  Grasp executor: interpolated IK empty, problem is with grasp location");
      if (marker_publisher_) marker_publisher_->colorGraspMarker(marker_id_, 1.0, 1.0, 0.0); //yellow
      if (error_code == ArmNavigationErrorCodes::COLLISION_CONSTRAINTS_VIOLATED)
	return Result(GraspResult::GRASP_IN_COLLISION, true);
      else if (error_code == ArmNavigationErrorCodes::JOINT_LIMITS_VIOLATED)
	return Result(GraspResult::GRASP_OUT_OF_REACH, true);
      else return Result(GraspResult::GRASP_UNFEASIBLE, true);
    }
    if (marker_publisher_) marker_publisher_->colorGraspMarker(marker_id_, 0.0, 1.0, 1.0); //cyan
    if (error_code == ArmNavigationErrorCodes::COLLISION_CONSTRAINTS_VIOLATED)
      return Result(GraspResult::PREGRASP_IN_COLLISION, true);
    else if (error_code == ArmNavigationErrorCodes::JOINT_LIMITS_VIOLATED)
      return Result(GraspResult::PREGRASP_OUT_OF_REACH, true);
    else return Result(GraspResult::PREGRASP_UNFEASIBLE, true);
  }

  //check if the first pose in trajectory is valid
  //when we check from pre-grasp to grasp we use custom link padding, so we need to check here
  //if the initial pose is feasible with default padding; otherwise, move_arm might refuse to
  //take us there
  if ( !mechInterface().checkStateValidity(pickup_goal.arm_name, grasp_trajectory.points.front().positions,
                                           pickup_goal.additional_collision_operations,
                                           pickup_goal.additional_link_padding) )
  {
    ROS_DEBUG("  Grasp executor: initial pose in grasp trajectory is unfeasible with default padding");
    return Result(GraspResult::PREGRASP_UNFEASIBLE, true);
  }

  return Result(GraspResult::SUCCESS, true);
}


GraspResult GraspExecutorWithApproach::prepareGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
						    const object_manipulation_msgs::Grasp &grasp)
{
  if (marker_publisher_) marker_publisher_->colorGraspMarker(marker_id_, 1.0, 0.0, 0.0); //red

  //demo_synchronizer::getClient().sync(3, "Computing interpolated IK path from pre-grasp to grasp to lift");
  //demo_synchronizer::getClient().rviz(3, "Collision models;Interpolated IK;IK contacts;Grasp execution");

  GraspResult result = getInterpolatedIKForGrasp(pickup_goal, grasp, interpolated_grasp_trajectory_);
  if ( result.result_code != GraspResult::SUCCESS )
  {
    ROS_DEBUG("  Grasp executor: failed to generate grasp trajectory");
    return result;
  }

  ROS_INFO("  Grasp executor: sucessfully computed IIK trac to grasp pose");

  //check for lift trajectory starting from grasp solution
  std::vector<double> grasp_joint_angles = interpolated_grasp_trajectory_.points.back().positions;
  result = getInterpolatedIKForLift(pickup_goal, grasp, grasp_joint_angles, interpolated_lift_trajectory_);
  if (result.result_code != GraspResult::SUCCESS)
  {
    ROS_DEBUG("  Grasp executor: failed to generate lift trajectory");
    return result;
  }

  ROS_INFO("  Grasp executor: sucessfully computed IIK to lift pose");

  return Result(GraspResult::SUCCESS, true);
}

GraspResult
GraspExecutorWithApproach::executeGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
					const object_manipulation_msgs::Grasp &grasp)
{
  //demo_synchronizer::getClient().sync(2, "Using motion planner to move arm to pre-grasp position");
  //demo_synchronizer::getClient().rviz(1, "Collision models;Planning goal;Environment contacts;Collision map");

 ROS_INFO("  Grasp executor: move to pre-grasp pose");
  if ( !mechInterface().attemptMoveArmToGoal(pickup_goal.arm_name,
					     interpolated_grasp_trajectory_.points.front().positions,
                                             pickup_goal.additional_collision_operations,
                                             pickup_goal.additional_link_padding) )
  {
    ROS_DEBUG("  Grasp executor: move_arm to pre-grasp reports failure");
    if (marker_publisher_) marker_publisher_->colorGraspMarker(marker_id_, 1.0, 0.5, 0.0); //orange-ish
    return Result(GraspResult::MOVE_ARM_FAILED, true);
  }
   ROS_INFO("  Grasp executor: move hand to pregrasp posture");
  mechInterface().handPostureGraspAction(pickup_goal.arm_name, grasp,
					 object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP);

  //demo_synchronizer::getClient().sync(2, "Executing interpolated IK path from pre-grasp to grasp");
  //demo_synchronizer::getClient().rviz(1, "Collision models");

  ROS_INFO("  Grasp executor: attempt iik trajactory");
  //execute the unnormalized interpolated trajectory from pre-grasp to grasp
  mechInterface().attemptTrajectory(pickup_goal.arm_name, interpolated_grasp_trajectory_, true);

  ROS_INFO("  Grasp executor: move hand to grasp posture");
  mechInterface().handPostureGraspAction(pickup_goal.arm_name, grasp,
					 object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP);

  if (marker_publisher_) marker_publisher_->colorGraspMarker(marker_id_, 0.0, 1.0, 0.0); //green
  return Result(GraspResult::SUCCESS, true);
}

/*! Retreats by the actual length of the pre-grasp to grasp trajectory (if any), or by the
  desired length of that path, if no path exists. Disables collision between gripper and object
  as well as table.
*/
GraspResult GraspExecutorWithApproach::retreat(const object_manipulation_msgs::PickupGoal &pickup_goal)
{
  motion_planning_msgs::OrderedCollisionOperations ord;
  motion_planning_msgs::CollisionOperation coll;
  //disable collision between gripper and object
  coll.object1 = handDescription().gripperCollisionName(pickup_goal.arm_name);
  coll.object2 = pickup_goal.collision_object_name;
  coll.operation = motion_planning_msgs::CollisionOperation::DISABLE;
  ord.collision_operations.push_back(coll);
  //disable collision between gripper and table
  coll.object2 = pickup_goal.collision_support_surface_name;
  ord.collision_operations.push_back(coll);
  ord.collision_operations = concat(ord.collision_operations,
                                    pickup_goal.additional_collision_operations.collision_operations);

  geometry_msgs::Vector3Stamped direction;
  direction.header.stamp = ros::Time::now();
  direction.header.frame_id = handDescription().gripperFrame(pickup_goal.arm_name);
  direction.vector = mechInterface().negate( handDescription().approachDirection(pickup_goal.arm_name) );

  //even if the complete retreat trajectory is not possible, execute as many
  //steps as we can (pass min_distance = 0)
  float retreat_distance = pickup_goal.min_approach_distance;
  float actual_distance;
  if (!mechInterface().translateGripper(pickup_goal.arm_name, direction,
					ord, pickup_goal.additional_link_padding,
                                        retreat_distance, 0, actual_distance))
  {
    ROS_ERROR(" Grasp executor: failed to retreat gripper at all");
    return Result(GraspResult::RETREAT_FAILED, false);
  }
  if (actual_distance < retreat_distance)
  {
    ROS_WARN(" Grasp executor: only partial retreat (%f) succeeeded", actual_distance);
    return Result(GraspResult::RETREAT_FAILED, false);
  }
  return Result(GraspResult::SUCCESS, true);
}

} //namespace object_manipulator
