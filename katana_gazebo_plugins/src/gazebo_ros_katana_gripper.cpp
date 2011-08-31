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

#include <katana_gazebo_plugins/gazebo_ros_katana_gripper.h>
#include <sensor_msgs/JointState.h>

#include <gazebo/Joint.hh>
//#include <gazebo/Body.hh>
//#include <gazebo/Geom.hh>
#include <gazebo/Simulator.hh>
//#include <gazebo/Entity.hh>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
//#include <gazebo/XMLConfig.hh>

#include <ros/time.h>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_katana_gripper", GazeboRosKatanaGripper)

GazeboRosKatanaGripper::GazeboRosKatanaGripper(Entity *parent) :
  Controller(parent), publish_counter_(0)
{
  ros::MultiThreadedSpinner s(1);
  boost::thread spinner_thread(boost::bind(&ros::spin, s));

  my_parent_ = dynamic_cast<Model*> (parent);

  if (!my_parent_)
    gzthrow("Gazebo_ROS_Katana_Gripper controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  node_namespaceP_ = new ParamT<std::string> ("node_namespace", "katana", 0);
  joint_nameP_.push_back(new ParamT<std::string> ("r_finger_joint", "katana_r_finger_joint", 1));
  joint_nameP_.push_back(new ParamT<std::string> ("l_finger_joint", "katana_l_finger_joint", 1));
  torqueP_ = new ParamT<float> ("max_torque", 10.0, 1);
  Param::End();

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    joints_[i] = NULL;
  }

}

GazeboRosKatanaGripper::~GazeboRosKatanaGripper()
{
  delete torqueP_;
  delete node_namespaceP_;
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    delete joint_nameP_[i];
  }
  delete rosnode_;
}

void GazeboRosKatanaGripper::LoadChild(XMLConfigNode *node)
{
  node_namespaceP_->Load(node);
  torqueP_->Load(node);
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    joint_nameP_[i]->Load(node);
    joints_[i] = my_parent_->GetJoint(**joint_nameP_[i]);
    if (!joints_[i])
      gzthrow("The controller couldn't get a joint");
  }

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_ros_katana_gripper", ros::init_options::NoSigintHandler
        | ros::init_options::AnonymousName);
  }

  rosnode_ = new ros::NodeHandle(**node_namespaceP_);

  //  joint_state_pub_ = rosnode_->advertise<sensor_msgs::JointState> ("/joint_states", 1);
  controller_state_pub_ = rosnode_->advertise<katana_msgs::GripperControllerState> ("gripper_controller_state", 1);

  //  for (size_t i = 0; i < NUM_JOINTS; ++i)
  //  {
  //    js_.name.push_back(**joint_nameP_[i]);
  //    js_.position.push_back(0);
  //    js_.velocity.push_back(0);
  //    js_.effort.push_back(0);
  //  }

  // construct pid controller
  if (!pid_controller_.init(ros::NodeHandle(*rosnode_, "gripper_pid")))
  {
    ROS_FATAL("gazebo_ros_katana_gripper could not construct PID controller!");
  }
  // TODO: add katana_gripper_grasp_controller

}

void GazeboRosKatanaGripper::InitChild()
{
  pid_controller_.reset();
  prev_update_time_ = Simulator::Instance()->GetSimTime();
}

void GazeboRosKatanaGripper::FiniChild()
{
  rosnode_->shutdown();
}

void GazeboRosKatanaGripper::UpdateChild()
{
  // --------------- command joints  ---------------
  Time step_time = Simulator::Instance()->GetSimTime() - prev_update_time_;
  prev_update_time_ = Simulator::Instance()->GetSimTime();
  ros::Duration dt = ros::Duration(step_time.Double());

  double desired_pos[NUM_JOINTS];
  double actual_pos[NUM_JOINTS];
  double commanded_effort[NUM_JOINTS];

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    // desired_pos = 0.3 * sin(0.25 * Simulator::Instance()->GetSimTime());
    //if ((prev_update_time_.sec / 6) % 2 == 0)
    //  desired_pos[i] = 0.3;
    //else
    //  desired_pos[i] = -0.44;

    desired_pos[i] = gripper_grasp_controller_.getDesiredAngle();
    actual_pos[i] = joints_[i]->GetAngle(0).GetAsRadian();

    commanded_effort[i] = pid_controller_.updatePid(actual_pos[i] - desired_pos[i], joints_[i]->GetVelocity(0), dt);

    joints_[i]->SetForce(0, commanded_effort[i]);

    // TODO: do I really have to set this every time?
    joints_[i]->SetMaxForce(0, **(torqueP_));

    // TODO: ensure that both joints are always having (approximately)
    // the same joint position, even if one is blocked by an object
  }

  // --------------- update gripper_grasp_controller  ---------------
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    gripper_grasp_controller_.setCurrentAngle(joints_[i]->GetAngle(0).GetAsRadian());
  }

  // --------------- limit publishing frequency to 25 Hz ---------------
  publish_counter_ = ((publish_counter_ + 1) % 40);

  if (publish_counter_ == 0)
  {
    // --------------- publish gripper controller state  ---------------
    katana_msgs::GripperControllerState controller_state;
    for (size_t i = 0; i < NUM_JOINTS; ++i)
    {
      controller_state.actual.push_back(actual_pos[i]);
      controller_state.desired.push_back(desired_pos[i]);
      controller_state.error.push_back(desired_pos[i] - actual_pos[i]);
    }

    controller_state_pub_.publish(controller_state);

    // don't publish joint states: The pr2_controller_manager publishes joint states for
    // ALL joints, not just the ones it controls.
    //
    //    // --------------- publish joint states ---------------
    //    js_.header.stamp = ros::Time::now();
    //
    //    for (size_t i = 0; i < NUM_JOINTS; ++i)
    //    {
    //      js_.position[i] = joints_[i]->GetAngle(0).GetAsRadian();
    //      js_.velocity[i] = joints_[i]->GetVelocity(0);
    //      js_.effort[i] = commanded_effort[i];
    //
    //      ROS_DEBUG("publishing gripper joint state %d (effort: %f)", i, commanded_effort[i]);
    //    }
    //
    //    joint_state_pub_.publish(js_);
  }
}
