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
 * katana_gripper_grasp_controller.cpp
 *
 *  Created on: 29.01.2011
 *      Author: Martin Günther <mguenthe@uos.de>
 *
 * based on pr2_gripper_grasp_controller
 *
 */

#include <katana/katana_gripper_grasp_controller.h>

namespace katana
{

KatanaGripperGraspController::KatanaGripperGraspController(boost::shared_ptr<AbstractKatana> katana) :
  katana_(katana), last_command_was_close_(false)
{
  ros::NodeHandle root_nh("");
  ros::NodeHandle pn("~");

  pn.param<double> ("gripper_object_presence_threshold", gripper_object_presence_threshold_,
                    DEFAULT_GRIPPER_OBJECT_PRESENCE_THRESHOLD);

  std::string query_service_name = root_nh.resolveName("gripper_grasp_status");
  query_srv_ = root_nh.advertiseService(query_service_name, &KatanaGripperGraspController::serviceCallback, this);
  ROS_INFO_STREAM("katana gripper grasp query service started on topic " << query_service_name);

  std::string gripper_grasp_posture_controller = root_nh.resolveName("gripper_grasp_posture_controller");
  action_server_ = new actionlib::SimpleActionServer<control_msgs::GripperCommandAction>(root_nh,
      gripper_grasp_posture_controller, boost::bind(&KatanaGripperGraspController::executeCB, this, _1), false);
  action_server_->start();
  ROS_INFO_STREAM("katana gripper grasp hand posture action server started on topic " << gripper_grasp_posture_controller);
}

KatanaGripperGraspController::~KatanaGripperGraspController()
{
  delete action_server_;
}

void KatanaGripperGraspController::executeCB(const control_msgs::GripperCommandGoalConstPtr &goal)
{
  if (goal->command.position < 0.01)
  {
    
    katana_->moveJoint(GRIPPER_INDEX, GRIPPER_CLOSED_ANGLE);

    last_command_was_close_ = true;
  }
  else if (goal->command.position > 0.10)
  {
    katana_->moveJoint(GRIPPER_INDEX, GRIPPER_OPEN_ANGLE);
    last_command_was_close_ = false;
  }
  else
  {
    katana_->moveJoint(GRIPPER_INDEX, acos((2*(GRIPPER_LENGTH * GRIPPER_LENGTH) - goal->command.position * 
                                                    goal->command.position))/(4*GRIPPER_LENGTH));
    // ROS_ERROR("katana gripper grasp execution: unknown goal code (%f)", goal->command.position);
    // action_server_->setAborted();
  }

  // wait for gripper to open/close
  ros::Duration(GRIPPER_OPENING_CLOSING_DURATION).sleep();

  // If we ever do anything else than setSucceeded() in the future, we will have to really
  // check if the desired opening angle was reached by the gripper here.
  //static const bool move_gripper_success = true;
  static bool move_gripper_success = true;
  // process the result
  if (move_gripper_success)
  {
    ROS_INFO("Gripper goal achieved");
    action_server_->setSucceeded();
  }
  else
  {
      ROS_WARN("Gripper goal not achieved for pre-grasp or release");
      // this might become a failure later, for now we're still investigating
      action_server_->setSucceeded();
  }
}

bool KatanaGripperGraspController::serviceCallback(control_msgs::QueryTrajectoryState::Request &request,
                                                   control_msgs::QueryTrajectoryState::Response &response)
{
  if (!last_command_was_close_)
  {
    ROS_INFO("Gripper grasp query false: the last gripper command was not 'close' (= not holding any object)");
    
    return true;
  }

  double gripperAngle = katana_->getMotorAngles()[GRIPPER_INDEX];
  if (gripperAngle < gripper_object_presence_threshold_)
  {
    ROS_INFO("Gripper grasp query false: gripper angle %f below threshold %f (= not holding any object)",
        gripperAngle, gripper_object_presence_threshold_);
  }
  else
  {
    ROS_INFO("Gripper grasp query true: gripper angle %f above threshold %f (= holding an object)",
        gripperAngle, gripper_object_presence_threshold_);
  }
  return true;
}

}
