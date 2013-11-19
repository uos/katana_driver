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
 * katana_gripper_grasp_controller.h
 *
 *  Created on: 29.01.2011
 *      Author: Martin Günther <mguenthe@uos.de>
 *
 * based on pr2_gripper_grasp_controller
 */

#ifndef KATANA_GRIPPER_GRASP_CONTROLLER_H_
#define KATANA_GRIPPER_GRASP_CONTROLLER_H_

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/QueryTrajectoryState.h>

#include <katana/AbstractKatana.h>


namespace katana
{

class KatanaGripperGraspController
{
public:
  KatanaGripperGraspController(boost::shared_ptr<AbstractKatana> katana);
  virtual ~KatanaGripperGraspController();

private:
  //! Action server for the grasp posture action
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> *action_server_;

  //! Server for the posture query service
  ros::ServiceServer query_srv_;

  boost::shared_ptr<AbstractKatana> katana_;

  //! A joint angle below this value indicates there is nothing inside the gripper
  double gripper_object_presence_threshold_;

  //! A difference in desired goal angle and actual goal angle above this value indicates that the goal was not reached
  double goal_threshold_;

  void executeCB(const control_msgs::GripperCommandGoalConstPtr &goal);

  bool serviceCallback(control_msgs::QueryTrajectoryState::Request &request,
                       control_msgs::QueryTrajectoryState::Response &response);

};

}

#endif /* KATANA_GRIPPER_GRASP_CONTROLLER_H_ */
