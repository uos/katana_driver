/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2011  University of Osnabrück
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * katana_gripper_joint_trajectory_controller.h
 *
 *  Created on: 20.10.2011
 *      Author: Karl Glatz - Ravensburg-Weingarten, University of Applied Sciences
 *
 *  based on joint_trajectory_action/src/joint_trajectory_action.cpp
 *
 */

#include <katana_gazebo_plugins/katana_gripper_joint_trajectory_controller.h>

namespace katana_gazebo_plugins
{

/*node_(private_node),*/
KatanaGripperJointTrajectoryController::KatanaGripperJointTrajectoryController(ros::NodeHandle pn) :
    has_active_goal_(false), desired_angle_(0.0), current_angle_(0.0)
{


  // set the joints fixed here
  joint_names_.push_back((std::string)"r_finger_joint"); // katana_r_finger_joint
  joint_names_.push_back((std::string)"l_finger_joint"); // katana_l_finger_joint

  pn.param("constraints/goal_time", goal_time_constraint_, 0.0);

  // Gets the constraints for each joint.
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    std::string ns = std::string("constraints/") + joint_names_[i];
    double g, t;
    pn.param(ns + "/goal", g, -1.0);
    pn.param(ns + "/trajectory", t, -1.0);
    goal_constraints_[joint_names_[i]] = g;
    trajectory_constraints_[joint_names_[i]] = t;
  }
  pn.param("constraints/stopped_velocity_tolerance", stopped_velocity_tolerance_, 0.01);

  ros::NodeHandle action_node("katana_arm_controller");

  //TODO: name needs to be the same as for the real katana ?? Or is it just a matter of the joint names inside of the goal?
  action_server_ = new JTAS(action_node, "gripper_joint_trajectory_action",
                            boost::bind(&KatanaGripperJointTrajectoryController::goalCB, this, _1),
                            boost::bind(&KatanaGripperJointTrajectoryController::cancelCB, this, _1), false);

  action_server_->start();
  ROS_INFO(
      "katana gripper joint trajctory action server started on topic katana_arm_controller/gripper_joint_trajectory_action");


}

KatanaGripperJointTrajectoryController::~KatanaGripperJointTrajectoryController()
{
  delete action_server_;
}

bool KatanaGripperJointTrajectoryController::setsEqual(const std::vector<std::string> &a,
                                                       const std::vector<std::string> &b)
{
  if (a.size() != b.size())
    return false;

  for (size_t i = 0; i < a.size(); ++i)
  {
    if (count(b.begin(), b.end(), a[i]) != 1)
      return false;
  }
  for (size_t i = 0; i < b.size(); ++i)
  {
    if (count(a.begin(), a.end(), b[i]) != 1)
      return false;
  }

  return true;
}

void KatanaGripperJointTrajectoryController::checkGoalStatus()
{

  ros::Time now = ros::Time::now();

  if (!has_active_goal_)
    return;
  if (current_traj_.points.empty())
    return;

  // time left?
  if (now < current_traj_.header.stamp + current_traj_.points[0].time_from_start)
    return;

  int last = current_traj_.points.size() - 1;
  ros::Time end_time = current_traj_.header.stamp + current_traj_.points[last].time_from_start;

  bool inside_goal_constraints = false;

  if (desired_angle_queue_.empty())
  {

    if (this->currentIsDesiredAngle())
    {
      inside_goal_constraints = true;
    }

  }

  if (inside_goal_constraints)
  {
    ROS_DEBUG("Goal Succeeded!");
    active_goal_.setSucceeded();
    has_active_goal_ = false;
  }
  else if (now < end_time + ros::Duration(goal_time_constraint_))
  {
    // Still have some time left to make it.
    ROS_DEBUG("Still have some time left to make it.");
  }
  else
  {
    ROS_WARN("Aborting because we wound up outside the goal constraints");
    active_goal_.setAborted();
    has_active_goal_ = false;
  }

}

bool KatanaGripperJointTrajectoryController::currentIsDesiredAngle()
{

  ROS_DEBUG("current_angle_: %f desired_angle_: %f", current_angle_, desired_angle_);

  return ((current_angle_ - GRIPPER_ANGLE_THRESHOLD) <= desired_angle_
      && (current_angle_ + GRIPPER_ANGLE_THRESHOLD) >= desired_angle_);

}

void KatanaGripperJointTrajectoryController::goalCB(GoalHandle gh)
{

  ROS_DEBUG("KatanaGripperJointTrajectoryController::goalCB");

  // Ensures that the joints in the goal match the joints we are commanding.
  if (!setsEqual(joint_names_, gh.getGoal()->trajectory.joint_names))
  {
    ROS_ERROR("KatanaGripperJointTrajectoryController::goalCB: Joints on incoming goal don't match our joints");
    gh.setRejected();
    return;
  }

  // Cancels the currently active goal.
  if (has_active_goal_)
  {
    // Stops the controller.
    this->stopController();

    // Marks the current goal as canceled.
    active_goal_.setCanceled();
    has_active_goal_ = false;
  }

  gh.setAccepted();
  active_goal_ = gh;
  has_active_goal_ = true;

  // Sends the trajectory along to the controller
  current_traj_ = active_goal_.getGoal()->trajectory;

  this->publish(current_traj_);
}

void KatanaGripperJointTrajectoryController::cancelCB(GoalHandle gh)
{
  if (active_goal_ == gh)
  {
    // Stops the controller.
    this->stopController();

    // Marks the current goal as canceled.
    active_goal_.setCanceled();
    has_active_goal_ = false;
  }
}


void KatanaGripperJointTrajectoryController::publish(trajectory_msgs::JointTrajectory traj)
{

  if (traj.points.empty())
  {
    ROS_WARN("KatanaGripperJointTrajectoryController::publish: Empty trajectory");
    return;
  }

  size_t numof_points = traj.points.size();
  for (size_t i = 1; i < numof_points; i++)
  {
    // ensure there are 2 positions in each point
    if (traj.points[i].positions.size() != 2)
    {
      ROS_WARN("Trajectory contains positions with more or less than 2 positions");
      continue;
    }

    size_t start_index = i - 1;
    size_t end_index = i;

    // use first point, because both must be the same!
    // TODO: any check of equality?
    double start_pos = traj.points[start_index].positions[0];
    double start_vel = traj.points[start_index].velocities[0];
    double start_acc = traj.points[start_index].accelerations[0];

    double end_pos = traj.points[end_index].positions[0];
    double end_vel = traj.points[end_index].velocities[0];
    double end_acc = traj.points[end_index].accelerations[0];

    double time_from_start = traj.points[end_index].time_from_start.toSec();

    std::vector<double> coefficients;

    spline_smoother::getCubicSplineCoefficients(start_pos, start_vel, end_pos, end_vel, time_from_start, coefficients);

    for (double t = 0.0; t < time_from_start; t = t + GRIPPER_SAMPLING_TIME_STEPS)
    {
      double sample_pos, sample_vel, sample_acc;
      spline_smoother::sampleQuinticSpline(coefficients, t, sample_pos, sample_vel, sample_acc);

      ROS_DEBUG("point %i: time %f sample_pos %f to queue", i, t, sample_pos);
      this->desired_angle_queue_.push_back(sample_pos);
    }

  }

}

void KatanaGripperJointTrajectoryController::stopController()
{

  desired_angle_queue_.clear();
}

}
