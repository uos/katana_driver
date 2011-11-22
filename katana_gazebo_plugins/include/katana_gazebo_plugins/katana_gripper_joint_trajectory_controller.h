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
 *      Author: Karl Glatz <glatz@hs-weingarten.de>
 *              Ravensburg-Weingarten, University of Applied Sciences
 *
 *  based on joint_trajectory_action/src/joint_trajectory_action.cpp
 *
 */
#ifndef KATANA_GRIPPER_JOINT_TRAJECTORY_CONTROLLER_H_
#define KATANA_GRIPPER_JOINT_TRAJECTORY_CONTROLLER_H_

#include <deque>

#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <spline_smoother/splines.h>

//#include <katana/spline_functions.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

#include <katana_gazebo_plugins/gazebo_ros_katana_gripper_action_interface.h>

namespace katana_gazebo_plugins
{

/**
 *  allowed difference between desired and actual position
 */
static const double GRIPPER_ANGLE_THRESHOLD = 0.03;
/**
 *  sampling time setps in seconds
 */
static const double GRIPPER_SAMPLING_TIME_STEPS = 0.01;

/**
 * This class allows you to send JointTrajectory messages to the Katana Arm simulated in Gazebo
 */
class KatanaGripperJointTrajectoryController : public IGazeboRosKatanaGripperAction
{

private:
  typedef actionlib::ActionServer<pr2_controllers_msgs::JointTrajectoryAction> JTAS;
  typedef JTAS::GoalHandle GoalHandle;

public:

  KatanaGripperJointTrajectoryController(ros::NodeHandle pn);
  virtual ~KatanaGripperJointTrajectoryController();

private:

  JTAS *action_server_;

  bool has_active_goal_;
  GoalHandle active_goal_;
  trajectory_msgs::JointTrajectory current_traj_;
  bool trajectory_finished_;

  // the internal state of the gripper
  GRKAPoint current_point_;
  GRKAPoint last_desired_point_;
//  std::deque<GRKAPoint> desired_points_queue_;

  std::vector<std::string> joint_names_;
  std::map<std::string, double> goal_constraints_;
  std::map<std::string, double> trajectory_constraints_;
  double goal_time_constraint_;
  double stopped_velocity_tolerance_;

  // call-back methods
  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);

  // helper methods
  static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b);
  void checkGoalStatus();
  bool currentIsDesiredAngle();
  void setCurrentTrajectory(trajectory_msgs::JointTrajectory traj);
//  void clearQueue();
//  bool isEmptyQueue();
  bool isTrajectoryFinished();

public:
  // public methods defined by interface IGazeboRosKatanaGripperAction

  GRKAPoint getNextDesiredPoint(ros::Time time);
//  {
//    // get next value out of the list
//    if (!desired_points_queue_.empty())
//    {
//      // get first element in queue
//      last_desired_point_ = desired_points_queue_.front();
//      // remove the first element
//      desired_points_queue_.pop_front();
//    }
//
//    return last_desired_point_;
//  }

  void setCurrentPoint(GRKAPoint point)
  {
    this->current_point_ = point;
    this->checkGoalStatus();
  }

  void cancelGoal()
  {
//    if (has_active_goal_)
//    {
    cancelCB(active_goal_);
//    }
//    else
//    {
//      this->clearQueue();
//    }

  }

  /**
   * are there any more points?
   */
  bool hasActiveGoal() const
  {
    return has_active_goal_;
  }

};

}

#endif /* KATANA_GRIPPER_JOINT_TRAJECTORY_CONTROLLER_H_ */
