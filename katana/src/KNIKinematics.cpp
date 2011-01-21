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
 * KNIKinematics.cpp
 *
 *  Created on: 20.01.2011
 *      Author: Martin Günther
 */

#include "../include/katana/KNIKinematics.h"

namespace katana
{

KNIKinematics::KNIKinematics(boost::shared_ptr<AbstractKatana> katana) :
  katana(katana)
{
  ros::NodeHandle nh;

  get_kinematic_solver_info_server_ = nh.advertiseService("get_kinematic_solver_info",
                                                          &KNIKinematics::get_kinematic_solver_info, this);

  // TODO: register GetPositionFK, GetPositionIK
}

KNIKinematics::~KNIKinematics()
{
}

void KNIKinematics::loopOnce()
{
  ros::spinOnce();
}

bool KNIKinematics::get_kinematic_solver_info(kinematics_msgs::GetKinematicSolverInfo::Request &req,
                                              kinematics_msgs::GetKinematicSolverInfo::Response &res)
{
  res.kinematic_solver_info.joint_names = katana->getJointNames();
  return true;
}

bool KNIKinematics::get_position_fk(kinematics_msgs::GetPositionFK::Request &req,
                                    kinematics_msgs::GetPositionFK::Response &res)
{
  std::vector<double> jointAngles, pose;

  if (req.fk_link_names.size() != 1 || req.fk_link_names[0] != "katana_gripper_tool_frame")
  {
    ROS_ERROR("The KNI kinematics solver can only solve requests for katana_gripper_tool_frame!");
    return false;
  }

  // TODO: make joints lookup

  // TODO: put joint angles from req.robot_state in the correct order into jointAngles

  pose = katana->getCoordinates(jointAngles);

  // The frame_id in the header message is the frame in which
  // the forward kinematics poses will be returned; TODO: transform result to header frame_id

  // TODO: store in res
  return true;
}

///// copied from joint_trajectory_action_controller
//std::vector<int> KNIKinematics::makeJointsLookup(const trajectory_msgs::JointTrajectory &msg)
//{
//  std::vector<int> lookup(joints_.size(), -1); // Maps from an index in joints_ to an index in the msg
//  for (size_t j = 0; j < joints_.size(); ++j)
//  {
//    for (size_t k = 0; k < msg.joint_names.size(); ++k)
//    {
//      if (msg.joint_names[k] == joints_[j])
//      {
//        lookup[j] = k;
//        break;
//      }
//    }
//
//    if (lookup[j] == -1)
//    {
//      ROS_ERROR("Unable to locate joint %s in the commanded trajectory.", joints_[j].c_str());
//      return std::vector<int>(); // return empty vector to signal error
//    }
//  }
//
//  return lookup;
//}


}
