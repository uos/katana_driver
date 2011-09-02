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
 * joint_movement_adapter.cpp
 *
 *  Created on: 30.08.2011
 *      Author: Martin Günther <mguenthe@uos.de>
 *
 *  based on joint_trajectory_generator by Eitan Marder-Eppstein
 */

#include <katana_joint_movement_adapter/joint_movement_adapter.h>
#include <fstream>
#include <iostream>
#include <cstdio>

namespace katana_joint_movement_adapter
{

JointMovementAdapter::JointMovementAdapter(std::string name) :
  as_(ros::NodeHandle(), "joint_movement_action", boost::bind(&JointMovementAdapter::executeCB, this, _1), false),
      ac_("joint_trajectory_action"), got_state_(false)
{
  ros::NodeHandle n;
  state_sub_ = n.subscribe("state", 1, &JointMovementAdapter::jointStateCb, this);

  ros::NodeHandle pn("~");
  pn.param("max_acc", max_acc_, 0.5);
  pn.param("max_vel", max_vel_, 5.0);

  // Load Robot Model
  ROS_DEBUG("Loading robot model");
  std::string xml_string;
  ros::NodeHandle nh_toplevel;
  if (!nh_toplevel.getParam(std::string("/robot_description"), xml_string))
    throw ros::Exception("Could not find parameter robot_description on parameter server");

  if (!robot_model_.initString(xml_string))
    throw ros::Exception("Could not parse robot model");

  ros::Rate r(10.0);
  while (!got_state_)
  {
    ros::spinOnce();
    r.sleep();
  }

  //  gripper_joints_.push_back("katana_r_finger_joint");
  //  gripper_joints_.push_back("katana_l_finger_joint");

  ac_.waitForServer();
  as_.start();
  ROS_INFO("%s: Initialized",name.c_str());
}

JointMovementAdapter::~JointMovementAdapter()
{
}

void JointMovementAdapter::jointStateCb(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr& state)
{
  boost::mutex::scoped_lock lock(mutex_);
  for (unsigned int i = 0; i < state->joint_names.size(); ++i)
  {
    current_state_[state->joint_names[i]] = state->actual.positions[i];
  }
  got_state_ = true;
}

void JointMovementAdapter::executeCB(const JMAS::GoalConstPtr &movement_goal)
{
  ROS_DEBUG("Got a goal");

  // note: the SimpleActionServer guarantees that we enter this function only when
  // there is no other active goal. in other words, only one instance of executeCB()
  // is ever running at the same time.

  // ---------- adjust all goal positions to match the given motor limits ----------
  sensor_msgs::JointState limited_joint_states = limitJointStates(movement_goal->jointGoal);

  // ---------- transform JointMovement goal to rough JointTrajectory goal ----------
  pr2_controllers_msgs::JointTrajectoryGoal rough_trajectory_goal = makeRoughTrajectory(limited_joint_states);

  // ---------- generate full trajectory ----------
  pr2_controllers_msgs::JointTrajectoryGoal full_trajectory_goal;
  try
  {
    full_trajectory_goal = makeFullTrajectory(rough_trajectory_goal);
  }
  catch (ros::Exception ex)
  {
    ROS_ERROR_STREAM(ex.what());
    as_.setAborted();
    return;
  }

  // ---------- send goal to action client ----------
  ac_.sendGoal(full_trajectory_goal);

  // ---------- wait for action client to finish----------
  while (ros::ok() && !ac_.waitForResult(ros::Duration(0.05)))
  {
    if (as_.isPreemptRequested())
    {
      ROS_DEBUG("Preempted");
      ac_.cancelGoal();
    }
  }

  // ---------- check if node was killed ----------
  if (!ros::ok())
  {
    as_.setAborted();
    return;
  }

  // ---------- pass on terminal state from action client to action server ----------
  actionlib::SimpleClientGoalState state = ac_.getState();

  if (state == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    ROS_DEBUG("Preempted");
    as_.setPreempted();
  }
  else if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_DEBUG("Succeeded");
    as_.setSucceeded();
  }
  else if (state == actionlib::SimpleClientGoalState::ABORTED)
  {
    ROS_DEBUG("Aborted");
    as_.setAborted();
  }
  else
    as_.setAborted(katana_msgs::JointMovementResult(), "Unknown result from joint_trajectory_action");
}

pr2_controllers_msgs::JointTrajectoryGoal JointMovementAdapter::makeRoughTrajectory(
                                                                                    const sensor_msgs::JointState &jointGoal)
{
  pr2_controllers_msgs::JointTrajectoryGoal result;

  if (jointGoal.name.size() != jointGoal.position.size())
    ROS_FATAL("joint goal: name and position array have different size!");

  // copy current state to make sure every joint has a target
  std::map<std::string, double> target_state = std::map<std::string, double>(current_state_);

  // overwrite with positions from joint goal (can be only some of the joints)
  for (size_t i = 0; i < jointGoal.name.size(); ++i)
  {
    if (target_state.find(jointGoal.name[i]) == target_state.end())
      ROS_WARN("joint name %s is not one of our controlled joints, ignoring", jointGoal.name[i].c_str());
    else
      target_state[jointGoal.name[i]] = jointGoal.position[i];
  }

  // create a joint trajectory with only one trajectory point (the target);
  // the proper start point will be inserted by makeFullTrajectory()
  result.trajectory.header = jointGoal.header;
  result.trajectory.points.resize(1);

  // set the time at which the target should be reached to 0.0 seconds after start;
  // this will be adjusted by makeFullTrajectory()
  result.trajectory.points[0].time_from_start = ros::Duration(0.0);

  // copy positions to target
  for (std::map<std::string, double>::iterator it = target_state.begin(); it != target_state.end(); ++it)
  {
    result.trajectory.joint_names.push_back(it->first);
    result.trajectory.points[0].positions.push_back(it->second);

    // we want the arm to stop at the target (ignore velocities and accelerations from joint goal);
    // this is a requirement for the applicability of makeFullTrajectory().
    result.trajectory.points[0].velocities.push_back(0.0);
    result.trajectory.points[0].accelerations.push_back(0.0);
  }

  return result;
}

pr2_controllers_msgs::JointTrajectoryGoal JointMovementAdapter::makeFullTrajectory(
                                                                                   const pr2_controllers_msgs::JointTrajectoryGoal& goal)
{
  pr2_controllers_msgs::JointTrajectoryGoal new_goal;
  new_goal.trajectory.header = goal.trajectory.header;
  new_goal.trajectory.joint_names = goal.trajectory.joint_names;

  size_t n_traj_points = goal.trajectory.points.size(), n_joint_names = goal.trajectory.joint_names.size();

  // Increase traj length to account for the initial pose
  ROS_DEBUG_STREAM("Initial trajectory has "<<n_traj_points<<" points.");
  new_goal.trajectory.points.resize(n_traj_points + 1);

  // Set joint names
  for (size_t i = 0; i < n_traj_points + 1; i++)
  {
    new_goal.trajectory.points[i].positions.resize(n_joint_names);
  }

  {
    boost::mutex::scoped_lock lock(mutex_);
    //add the current point as the start of the trajectory
    for (unsigned int i = 0; i < n_joint_names; ++i)
    {
      // Generate the first point
      if (current_state_.find(new_goal.trajectory.joint_names[i]) == current_state_.end())
      {
        ROS_FATAL_STREAM("Joint names in goal and controller don't match. Something is very wrong. Goal joint name: "<<new_goal.trajectory.joint_names[i]);
        throw std::runtime_error("Joint names in goal and controller don't match. Something is very wrong.");
      }
      new_goal.trajectory.points[0].positions[i] = current_state_[new_goal.trajectory.joint_names[i]];

      // Get the joint and calculate the offset from the current state
      boost::shared_ptr<const urdf::Joint> joint = robot_model_.getJoint(new_goal.trajectory.joint_names[i]);
      double offset = 0;

      double goal_position = goal.trajectory.points[0].positions[i], current_position =
          new_goal.trajectory.points[0].positions[i];

      if (joint->type == urdf::Joint::REVOLUTE)
      {
        offset = 0;
      }
      else if (joint->type == urdf::Joint::CONTINUOUS)
      {
        offset = current_position - goal_position - angles::shortest_angular_distance(goal_position, current_position);
      }
      else
      {
        ROS_WARN("Unknown joint type in joint trajectory. This joint might not be unwrapped properly. Supported joint types are urdf::Joint::REVOLUTE and urdf::Joint::CONTINUOUS");
        offset = 0;
      }

      // Apply offset to each point in the trajectory on this joint
      for (unsigned int j = 0; j < n_traj_points; j++)
      {
        new_goal.trajectory.points[j + 1].time_from_start = goal.trajectory.points[j].time_from_start;
        new_goal.trajectory.points[j + 1].positions[i] = goal.trajectory.points[j].positions[i] + offset;
      }
    }
  }

  // pass into trajectory generator here
  trajectory::TrajectoryGenerator g(max_vel_, max_acc_, new_goal.trajectory.joint_names.size());

  // do the trajectory generation
  g.generate(new_goal.trajectory, new_goal.trajectory);

  return new_goal;
}

sensor_msgs::JointState JointMovementAdapter::limitJointStates(const sensor_msgs::JointState &jointGoal)
{
  sensor_msgs::JointState adjustedJointGoal;

  adjustedJointGoal.name = jointGoal.name;
  adjustedJointGoal.position = jointGoal.position;

  //  for (size_t i = 0; i < jointGoal.name.size(); i++)
  //  {
  //    ROS_DEBUG("%s - min: %f - max: %f - curr: % f - req: %f", jointGoal.name[i].c_str(), getMotorLimitMin(jointGoal.name[i]), getMotorLimitMax(jointGoal.name[i]), katana_->getMotorAngles()[katana_->getJointIndex(jointGoal.name[i])], jointGoal.position[i]);
  //  }

  for (size_t i = 0; i < jointGoal.name.size(); i++)
  {
    boost::shared_ptr<const urdf::Joint> joint = robot_model_.getJoint(jointGoal.name[i]);
    if (!joint)
      ROS_FATAL("Joint %s not found in URDF!", jointGoal.name[i].c_str());

    if (jointGoal.position[i] < joint->limits->lower)
    {
      adjustedJointGoal.position[i] = joint->limits->lower;
    }

    if (jointGoal.position[i] > joint->limits->upper)
    {
      adjustedJointGoal.position[i] = joint->limits->upper;
    }
  }

  return adjustedJointGoal;
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_movement_adapter");
  katana_joint_movement_adapter::JointMovementAdapter jma(ros::this_node::getName());

  ros::spin();

  return 0;
}
