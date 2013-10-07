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
 * joint_movement_adapter.h
 *
 *  Created on: 30.08.2011
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#ifndef JOINT_MOVEMENT_ADAPTER_H_
#define JOINT_MOVEMENT_ADAPTER_H_

#include <ros/ros.h>

#include <vector>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <sensor_msgs/JointState.h>
#include <katana_msgs/JointMovementAction.h>

#include <arm_navigation_msgs/JointLimits.h>

#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

#include <angles/angles.h>
#include <urdf/model.h>
#include <urdf_model/joint.h>

#include <joint_trajectory_generator/trajectory_generation.h>

namespace katana_joint_movement_adapter
{

class JointMovementAdapter
{
  typedef actionlib::SimpleActionServer<katana_msgs::JointMovementAction> JMAS;
  typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> JTAC;

public:
  JointMovementAdapter(std::string name);
  virtual ~JointMovementAdapter();

  void executeCB(const JMAS::GoalConstPtr &goal);
  void jointStateCb(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr& state);

private:
  pr2_controllers_msgs::JointTrajectoryGoal makeRoughTrajectory(const sensor_msgs::JointState &jointGoal);
  pr2_controllers_msgs::JointTrajectoryGoal makeFullTrajectory(const pr2_controllers_msgs::JointTrajectoryGoal& goal);

  sensor_msgs::JointState limitJointStates(const sensor_msgs::JointState &jointGoal);

  JMAS as_;
  JTAC ac_;
  boost::mutex mutex_;
  std::map<std::string, double> current_state_;
  ros::Subscriber state_sub_;
  bool got_state_;
  double max_acc_, max_vel_;
  urdf::Model robot_model_;

  // std::vector<std::string> gripper_joints_;

  sensor_msgs::JointState movement_goal_;
};

}

#endif /* JOINT_MOVEMENT_ADAPTER_H_ */
