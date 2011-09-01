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
 * gazebo_ros_katana_gripper.cpp
 *
 *  Created on: 29.08.2011
 *      Author: Martin Günther <mguenthe@uos.de>
 */
#ifndef GAZEBO_ROS_KATANA_H
#define GAZEBO_ROS_KATANA_H

#include <katana_gazebo_plugins/katana_gripper_grasp_controller.h>
#include <gazebo/Controller.hh>
#include <gazebo/Model.hh>

// #include <sensor_msgs/JointState.h>
#include <katana_msgs/GripperControllerState.h>
#include <control_toolbox/pid.h>
#include <ros/ros.h>

namespace gazebo
{
class GazeboRosKatanaGripper : public Controller
{
public:
  GazeboRosKatanaGripper(gazebo::Entity *parent);
  virtual ~GazeboRosKatanaGripper();

  virtual void LoadChild(XMLConfigNode *node);
  virtual void InitChild();
  virtual void FiniChild();
  virtual void UpdateChild();

private:
  static const size_t NUM_JOINTS = 2;

  ros::NodeHandle *rosnode_;

  //  ros::Publisher joint_state_pub_;
  ros::Publisher controller_state_pub_;

  ParamT<std::string> *node_namespaceP_;
  std::vector<ParamT<std::string> *> joint_nameP_;

  ///Torque applied to the joints
  ParamT<float> *torqueP_;

  Model *my_parent_;

  control_toolbox::Pid pid_controller_;

  Joint *joints_[NUM_JOINTS];

  // sensor_msgs::JointState js_;

  // Simulation time of the last update
  Time prev_update_time_;

  katana_gazebo_plugins::KatanaGripperGraspController *gripper_grasp_controller_;

  short publish_counter_;
};
}
#endif
