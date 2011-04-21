/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2010  University of Osnabrück
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
 * joint_trajectory_action_controller.cpp
 *
 *  Created on: 07.04.2011
 *      Author: Henning Deeken <hdeeken@uos.de>
 *
 * based on joint_trajectory_action_controller.cpp by Stuart Glaser,
 * from the package robot_mechanism_controllers
 *
 * TODO header information
 */

#include "katana/joint_movement_action_controller.h"
#include <fstream>
#include <iostream>
#include <cstdio>

namespace katana
{

JointMovementActionController::JointMovementActionController(boost::shared_ptr<AbstractKatana> katana) :
  katana_(katana), movement_executing_(false),
  action_server_(ros::NodeHandle(), "joint_movement_action", boost::bind(&JointMovementActionController::executeCB, this, _1))

{
  ros::NodeHandle node_;

  joints_ = katana_->getJointNames();
  gripper_joints_ = katana_->getGripperJointNames();

}

JointMovementActionController::~JointMovementActionController()
{

}

/**
 * Checks if all joints in the joint goal match a among joints of the katana
 */
bool JointMovementActionController::suitableJointGoal(std::vector<std::string> &jointGoalNames){

  bool suitableJointGoal = true;

    for(size_t i = 0; i < jointGoalNames.size(); i++){

      bool exists = false;

      for(size_t j = 0; j < joints_.size(); j++){

          if(jointGoalNames[i] == joints_[j]) exists = true;

      }

      for(size_t k = 0; k < gripper_joints_.size(); k++){

        if(jointGoalNames[i]  == gripper_joints_[k]) exists = true;

      }

     if(exists == false) suitableJointGoal = false;

    }

  return suitableJointGoal;

}

void adjustJointGoalPositionsToMotorLimits(sensor_msgs::JointState jointGoal){

  for(size_t i = 0; i < jointGoal.name.size(); i++)
  {

    if(jointGoal.position[i] < katana_->getMotorLimitMin(jointGoal.name[i])){

      jointGoal.position[i] = katana_->getMotorLimitMin(jointGoal.name[i]);

      ROS_INFO("%s - requested JointGoalPosition: %f exceeded MotorLimit: %f  - adjusted JointGoalPosition to MotorLimit", jointGoal.name[i], jointGoal.position[i], katana_->getMotorLimitsMin(jointGoal.name[i]));

    }


    if(jointGoal.position[i] > katana_->getMotorLimitMax(jointGoal.name[i])){

          jointGoal.position[i] = katana_->getMotorLimitMax(jointGoal.name[i]);

          ROS_INFO("%s - requested JointGoalPosition: %f exceeded MotorLimit: %f  - adjusted JointGoalPosition to MotorLimit", jointGoal.name[i], jointGoal.position[i], katana_->getMotorLimitsMax(jointGoal.name[i]));

   }

  }

}


void JointMovementActionController::executeCB(const JMAS::GoalConstPtr &goal)
{

  if (!suitableJointGoal(goal->jointGoal.name))
  {
    ROS_ERROR("Joints on incoming goal don't match our joints/gripper_joints");

    for (size_t i = 0; i < goal->jointGoal.name.size(); i++)
    {
      ROS_INFO("  incoming joint %d: %s", (int)i, goal->jointGoal.name[i].c_str());
    }

    for (size_t i = 0; i < joints_.size(); i++)
    {
      ROS_INFO("  our joint      %d: %s", (int)i, joints_[i].c_str());
    }

    for (size_t i = 0; i < gripper_joints_.size(); i++)
        {
          ROS_INFO("  our gripper_joint      %d: %s", (int)i, gripper_joints_[i].c_str());
        }
    action_server_.setAborted();
    return;
  }

  if (action_server_.isPreemptRequested())
  {
    ROS_WARN("New action goal already seems to have been cancelled!");
    action_server_.setPreempted();
    return;
  }

  if (movement_executing_)
  {
    ROS_WARN("Already executing a movement!");
    // I don't know if this can even happen with the simple action server, but let's make sure...
  }


  movement_executing_ = true;

  // adjust all goal positions to match the given motor limits

  adjustJointGoalPositionsToMotorLimits(goal->jointGoal.position);

  ROS_INFO("Sending movement to Katana arm...");

  for(unsigned int i = 0; i < goal->jointGoal.name.size();i++){

    katana_->moveJoint(katana_->getJointIndex(goal->jointGoal.name[i]), goal->jointGoal.position[i]);

  }
}


}
