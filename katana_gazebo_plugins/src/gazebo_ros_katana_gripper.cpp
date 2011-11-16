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

  my_parent_ = dynamic_cast<Model*>(parent);

  if (!my_parent_)
    gzthrow("Gazebo_ROS_Katana_Gripper controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  node_namespaceP_ = new ParamT<std::string>("node_namespace", "katana", 0);
  joint_nameP_.push_back(new ParamT<std::string>("r_finger_joint", "katana_r_finger_joint", 1));
  joint_nameP_.push_back(new ParamT<std::string>("l_finger_joint", "katana_l_finger_joint", 1));
  torqueP_ = new ParamT<float>("max_torque", 0.5, 1);
  Param::End();

  // create gripper actions
  katana_gazebo_plugins::IGazeboRosKatanaGripperAction* gripper_grasp_controller_ =
      new katana_gazebo_plugins::KatanaGripperGraspController(ros::NodeHandle(**node_namespaceP_));
  katana_gazebo_plugins::IGazeboRosKatanaGripperAction* gripper_jt_controller_ =
      new katana_gazebo_plugins::KatanaGripperJointTrajectoryController(ros::NodeHandle(**node_namespaceP_));

  // "register" gripper actions
  gripper_action_list_.push_back(gripper_grasp_controller_);
  gripper_action_list_.push_back(gripper_jt_controller_);

  // set default action
  active_gripper_action_ = gripper_grasp_controller_;
  //active_gripper_action_ = gripper_jt_controller_;

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

  // delete all gripper actions
  for (std::size_t i = 0; i != gripper_action_list_.size(); i++)
  {
    /* delete object at pointer */
    delete gripper_action_list_[i];
  }

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
    ros::init(argc, argv, "gazebo_ros_katana_gripper",
              ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  }

  rosnode_ = new ros::NodeHandle(**node_namespaceP_);

  //  joint_state_pub_ = rosnode_->advertise<sensor_msgs::JointState> ("/joint_states", 1);
  controller_state_pub_ = rosnode_->advertise<katana_msgs::GripperControllerState>("gripper_controller_state", 1);

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
  double desired_vel[NUM_JOINTS];
  double actual_pos[NUM_JOINTS];
  double commanded_effort[NUM_JOINTS];

  // check for new goals, if found and change the active_gripper_action_
  this->updateActiveGripperAction();

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    // desired_pos = 0.3 * sin(0.25 * Simulator::Instance()->GetSimTime());
    //if ((prev_update_time_.sec / 6) % 2 == 0)
    //  desired_pos[i] = 0.3;
    //else
    //  desired_pos[i] = -0.44;

//    desired_pos[i] = gripper_grasp_controller_->getDesiredAngle();
    desired_pos[i] = active_gripper_action_->getNextDesiredPoint().position;
    desired_vel[i] = active_gripper_action_->getNextDesiredPoint().velocity;
    actual_pos[i] = joints_[i]->GetAngle(0).GetAsRadian();

    //TODO: use velocity
    commanded_effort[i] = pid_controller_.updatePid(actual_pos[i] - desired_pos[i], joints_[i]->GetVelocity(0), dt);

    joints_[i]->SetForce(0, commanded_effort[i]);

    // I set this every time just in case some other entity changed it
    joints_[i]->SetMaxForce(0, **(torqueP_));

    // TODO: ensure that both joints are always having (approximately)
    // the same joint position, even if one is blocked by an object
  }

  // --------------- update gripper_grasp_controller  ---------------
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {

    // update all actions
    for (std::size_t i = 0; i != gripper_action_list_.size(); i++)
    {
      gripper_action_list_[i]->setCurrentPoint(joints_[i]->GetAngle(0).GetAsRadian(), joints_[i]->GetVelocity(0));
    }

  }

  // --------------- limit publishing frequency to 25 Hz ---------------
  publish_counter_ = ((publish_counter_ + 1) % 40);

  if (publish_counter_ == 0)
  {
    // --------------- publish gripper controller state  ---------------
    katana_msgs::GripperControllerState controller_state;
    controller_state.header.stamp = ros::Time::now();
    for (size_t i = 0; i < NUM_JOINTS; ++i)
    {
      controller_state.name.push_back(joints_[i]->GetName());
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

/**
 * Checks for new goals, if found changes the active_gripper_action_ member
 */
void GazeboRosKatanaGripper::updateActiveGripperAction()
{
  //TODO: improve the selection of the action, maybe prefer newer started actions (but how to know?)
  //      atm the list gives some kind of priority, and it its impossible to cancel a goal
  //      by submitting a new one to another action (but you can cancel the goal via a message)

  // search for a new action if the current (or default on) is finished with its goal
  // if we cant find a new action, just use the current one
  if (!active_gripper_action_->hasActiveGoal())
  {

    // find a new action with a goal
    for (std::size_t i = 0; i != gripper_action_list_.size(); i++)
    {
      // just use the first found
      if (gripper_action_list_[i]->hasActiveGoal())
      {
        // change the active gripper action
        active_gripper_action_ = gripper_action_list_[i];
        break;
      }
    }

  }

}

