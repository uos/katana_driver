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
 * non_constraint_aware_ik_adapter.cpp
 *
 *  Created on: 19.5.2011
 *      Author: Henning Deeken <hdeeken@uos.de>
 */

/**
 * This node provides the get_constraint_aware_ik (kinematics_msgs/GetConstraintAwarePositionIK)
 * service. It just strips away all the constraint aware stuff and calls a the openrave IK service instead.
 *
 * Normally, get_constraint_aware_ik checks the space of potential IK solutions for a
 * solution that obeys all constraints (i.e., a solution that is not in self-collision or in collision
 * with the environment). That only really makes sense for a redundant (>6 DoF) robot arm. Since the
 * Katana only has 5 DoF, we only get at most one IK solution, so there is nothing to filter.
 *
 * If we return a solution that violates constraints - for instance, that is in self-collision - then
 * move_arm will abort with an error code.  But if the solution is valid, everything should work.
 *
 */

#include <ros/ros.h>

#include <urdf/joint.h>
#include <urdf/model.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <motion_planning_msgs/JointPath.h>
#include <motion_planning_msgs/JointPathPoint.h>
#include <motion_planning_msgs/JointLimits.h>
#include <orrosplanning/IK.h>

ros::ServiceClient client_;
std::vector<std::string> joint_names_;
std::vector<motion_planning_msgs::JointLimits> joint_limits_;

ros::ServiceServer ikService;
ros::ServiceServer infoService;

bool getOpenRaveIK(kinematics_msgs::GetConstraintAwarePositionIK::Request &req,
                   kinematics_msgs::GetConstraintAwarePositionIK::Response &resp)
{

  orrosplanning::IK srv;

  // converting the GetConstraintAwarePositionIK::Request to an proper orrosplanning::IK::Request

  srv.request.joint_state = req.ik_request.robot_state.joint_state;
  srv.request.pose_stamped = req.ik_request.pose_stamped;
  srv.request.iktype = "Translation3D";
  srv.request.filteroptions = 0;
  client_.call(srv);

  resp.solution.joint_state.header = srv.response.solutions.header;
  resp.solution.joint_state.name = srv.response.solutions.joint_names;

  if (srv.response.solutions.points.size() == 0)
  {
    ROS_ERROR("OpenRaveIK found no solutions!");
  }
  else
  {

    if (srv.response.solutions.points.size() > 1)
      ROS_INFO("OpenRaveIK found more than one solution, we'll take the first one...");

    resp.solution.joint_state.position.resize(srv.response.solutions.points[0].positions.size());

    for (size_t i = 0; 0 < srv.response.solutions.points[0].positions.size(); i++)
      resp.solution.joint_state.position[i] = srv.response.solutions.points[0].positions[i];

  }

  resp.error_code = srv.response.error_code;

  return true;
}

bool get_kinematic_solver_info(kinematics_msgs::GetKinematicSolverInfo::Request &req,
                               kinematics_msgs::GetKinematicSolverInfo::Response &res)
{
  res.kinematic_solver_info.joint_names = joint_names_;
  res.kinematic_solver_info.limits = joint_limits_;

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "katana_openrave_kinematics");
  ros::NodeHandle n;

  ros::NodeHandle pn("~");
  std::string ik_service;

  pn.param<std::string> ("ik_service", ik_service, "IK");

  client_ = n.serviceClient<orrosplanning::IK> (ik_service);

  joint_names_.resize(5);
  joint_limits_.resize(5);

  std::string robot_desc_string;

  if (!n.getParam("robot_description", robot_desc_string))
  {
    ROS_FATAL("Couldn't get a robot_description from the param server");
    return 0;
  }

  urdf::Model model;
  model.initString(robot_desc_string);

  XmlRpc::XmlRpcValue joint_names;

  // Gets all of the joints
  if (!n.getParam("katana_joints", joint_names))
  {
    ROS_ERROR("No joints given. (namespace: %s)", n.getNamespace().c_str());
  }
  if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Malformed joint specification.  (namespace: %s)", n.getNamespace().c_str());
  }
  if (joint_names.size() != 5)
  {
    ROS_ERROR("Wrong number of joints! was: %d, expected: %zu", joint_names.size(), 5);
  }
  for (size_t i = 0; i < 5; ++i)
  {
    XmlRpc::XmlRpcValue &name_value = joint_names[i];
    if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("Array of joint names should contain all strings.  (namespace: %s)",
          n.getNamespace().c_str());
    }

    joint_names_[i] = (std::string)name_value;
    joint_limits_[i].joint_name = (std::string)name_value;
    joint_limits_[i].min_position = model.getJoint(joint_names_[i])->limits->lower;
    joint_limits_[i].max_position = model.getJoint(joint_names_[i])->limits->upper;
  }

  ikService = pn.advertiseService("get_openrave_ik", getOpenRaveIK);
  infoService = pn.advertiseService("get_kinematic_solver_info", get_kinematic_solver_info);


  //ros::service::waitForService("get_openrave_ik");
  //ros::service::waitForService("get_kinematic_solver_info");

  ROS_INFO("katana_openrave_kinematics sucessfully established it's services");
  for (size_t i = 0; i < 5; ++i)
    ROS_INFO("%s",joint_limits_[i].joint_name.c_str());


  ros::spin();

  return 0;
}


