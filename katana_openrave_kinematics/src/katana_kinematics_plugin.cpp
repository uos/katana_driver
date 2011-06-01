/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// TODO: Adapt Copyright Info

/*
 * Author: Henning Deeken  // hdeeken@uos.de
 *
 * based on the pr2_arm_kinematics_plugin.cpp by Sachin Chitta
 */

#include <arm_kinematics_constraint_aware/arm_kinematics_constraint_aware_utils.h>
#include <katana_openrave_kinematics/katana_openrave_kinematics.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <algorithm>
#include <numeric>

#include <pluginlib/class_list_macros.h>

using namespace tf;
using namespace kinematics;
using namespace std;
using namespace ros;

//register KatanaKinematics as a KinematicsBase implementation
PLUGINLIB_DECLARE_CLASS(katana_openrave_kinematics, KatanaKinematicsPlugin, katana_openrave_kinematics::KatanaKinematicsPlugin, kinematics::KinematicsBase)

namespace katana_openrave_kinematics
{

KatanaKinematicsPlugin::KatanaKinematicsPlugin() :
  active_(false)
{
}

bool KatanaKinematicsPlugin::isActive()
{
  if (active_)
    return true;
  return false;
}

bool KatanaKinematicsPlugin::initialize(std::string name)
{

  urdf::Model robot_model;
  std::string tip_name, xml_string;
  ros::NodeHandle private_handle("~/" + name);
  dimension_ = 5;
  while (!arm_kinematics_constraint_aware::loadRobotModel(private_handle, robot_model, root_name_, tip_name, xml_string) && private_handle.ok())
  {
    ROS_ERROR("Could not load robot model. Are you sure the robot model is on the parameter server?");
    ros::Duration(0.5).sleep();
  }

  ROS_WARN("Root: %s Tip: %s", root_name_.c_str(), tip_name.c_str());

  kinematics_msgs::KinematicSolverInfo kinematic_info_;

  getUrdfInfo(robot_model, root_name_, tip_name, kinematic_info_);

  ROS_INFO("Advertising services");

  std::string fk_service;
  private_handle.param<std::string> ("fk_service", fk_service, "get_fk");
  fk_service_ = node_handle_.serviceClient<kinematics_msgs::GetPositionFK> (fk_service);

  std::string fk_info;
  private_handle.param<std::string> ("fk_info", fk_info, "get_fk_solver_info");
  fk_solver_info_service_ = node_handle_.serviceClient<kinematics_msgs::GetKinematicSolverInfo> (fk_info);

  std::string ik_service;
  private_handle.param<std::string> ("ik_service", ik_service, "IK");
  ik_service_ = node_handle_.serviceClient<orrosplanning::IK> (ik_service);

  if (!ros::service::waitForService(fk_info))
  {
    ROS_ERROR("Could not load fk info");
    active_ = false;
  }

  if (!ros::service::waitForService(fk_service))
  {
    ROS_ERROR("Could not load fk");
    active_ = false;
  }

  if (!ros::service::waitForService(ik_service))
  {
    ROS_ERROR("Could not load ik");
    active_ = false;
  }
  else
  {
    fk_solver_info_ = kinematic_info_;
    ik_solver_info_ = fk_solver_info_;

    for (unsigned int i = 0; i < fk_solver_info_.joint_names.size(); i++)
    {
      ROS_INFO("KatanaKinematics:: joint name: %s",fk_solver_info_.joint_names[i].c_str());
    }
    for (unsigned int i = 0; i < ik_solver_info_.link_names.size(); i++)
    {
      ROS_INFO("KatanaKinematics can solve IK for %s",ik_solver_info_.link_names[i].c_str());
    }
    for (unsigned int i = 0; i < fk_solver_info_.link_names.size(); i++)
    {
      ROS_INFO("KatanaKinematics can solve FK for %s",fk_solver_info_.link_names[i].c_str());
    }
    ROS_INFO("KatanaKinematicsPlugin::active for %s",name.c_str());
    active_ = true;
  }
  // ROS_ERROR("Giving Tool Frame: %s",ik_solver_info_.link_names[0].c_str());
  return active_;
}

bool KatanaKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state, std::vector<double> &solution,
                                           int &error_code)
{
  if (!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code = 1; //kinematics::SUCCESS;
    return false;
  }

  orrosplanning::IK srv;
  srv.request.manip_name = "arm";
  srv.request.joint_state.header.frame_id = root_name_;
  srv.request.joint_state.position = ik_seed_state;
  srv.request.pose_stamped.pose = ik_pose;
  srv.request.pose_stamped.header.frame_id = root_name_;
  srv.request.pose_stamped.header.stamp = ros::Time::now();
  srv.request.iktype = "translationdirection5d";
  srv.request.filteroptions = 0;
  ROS_DEBUG("Call OpenRave IK");
  ik_service_.call(srv);

  if (srv.response.error_code.val == srv.response.error_code.SUCCESS)
  {
    if (srv.response.solutions.points.size() >= 1)
    {
      ROS_DEBUG("OpenRave IK found several solutions, we discard all solutions despite the first one");
    }
    solution.resize(dimension_);
    solution = srv.response.solutions.points[0].positions;
    error_code = 1; //kinematics::SUCCESS
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");
    error_code = -2; //kinematics::NO_IK_SOLUTION;
    return false;
  }
}

bool KatanaKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state, const double &timeout,
                                              std::vector<double> &solution, int &error_code)
{
  if (!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code = -8; //kinematics::INACTIVE;
    return false;
  }

  orrosplanning::IK srv;
  srv.request.manip_name = "arm";
  srv.request.joint_state.header.frame_id = root_name_;
  srv.request.joint_state.position = ik_seed_state;
  srv.request.pose_stamped.pose = ik_pose;
  srv.request.pose_stamped.header.frame_id = root_name_;
  srv.request.pose_stamped.header.stamp = ros::Time::now();
  srv.request.iktype = "translationdirection5d";
  srv.request.filteroptions = 0;
  ROS_DEBUG("Call OpenRave IK");
  ik_service_.call(srv);

  if (srv.response.error_code.val == srv.response.error_code.SUCCESS)
  {
    if (srv.response.solutions.points.size() >= 1)
    {
      ROS_DEBUG("OpenRave IK found several solutions, we discard all despite the first one");
    }
    solution.resize(dimension_);
    solution = srv.response.solutions.points[0].positions;
    error_code = 1; //kinematics::SUCCESS;

    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");
    error_code = -2; //kinematics::NO_IK_SOLUTION;
    return false;
  }
}

void KatanaKinematicsPlugin::desiredPoseCallback(const std::vector<double>& ik_seed_state,
                                                 const geometry_msgs::Pose& ik_pose,
                                                 motion_planning_msgs::ArmNavigationErrorCodes& error_code)
{

  int int_error_code;
  desiredPoseCallback_(ik_pose, ik_seed_state, int_error_code);
  if (int_error_code)
    error_code.val = motion_planning_msgs::ArmNavigationErrorCodes::SUCCESS;
  else
    error_code.val = motion_planning_msgs::ArmNavigationErrorCodes::NO_IK_SOLUTION;
}

void KatanaKinematicsPlugin::jointSolutionCallback(const std::vector<double>& solution,
                                                   const geometry_msgs::Pose& ik_pose,
                                                   motion_planning_msgs::ArmNavigationErrorCodes& error_code)
{
  int int_error_code;
  solutionCallback_(ik_pose, solution, int_error_code);
  if (int_error_code > 0)
    error_code.val = motion_planning_msgs::ArmNavigationErrorCodes::SUCCESS;
  else
    error_code.val = motion_planning_msgs::ArmNavigationErrorCodes::NO_IK_SOLUTION;
}

bool KatanaKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state, const double &timeout,
                                              std::vector<double> &solution,
                                              const boost::function<void(const geometry_msgs::Pose &ik_pose,
                                                                         const std::vector<double> &ik_solution,
                                                                         int &error_code)> &desired_pose_callback,
                                              const boost::function<void(const geometry_msgs::Pose &ik_pose,
                                                                         const std::vector<double> &ik_solution,
                                                                         int &error_code)> &solution_callback,
                                              int &error_code_int)
{
  if (!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code_int = -8; // kinematics::INACTIVE;
    return false;
  }

  desiredPoseCallback_ = desired_pose_callback;
  solutionCallback_ = solution_callback;

  motion_planning_msgs::ArmNavigationErrorCodes error_code;
  // perform IK and check for callback suitability

  if (!desired_pose_callback.empty())
    desiredPoseCallback(ik_seed_state, ik_pose, error_code);
  if (error_code.val != error_code.SUCCESS)
  {
    ROS_DEBUG("An IK solution could not be found, because the constraints in desired_pose_callback are not matched");
    error_code_int = -2; // NO_IK_SOLUTION
    return false;
  }

  orrosplanning::IK srv;
  std::vector<double> solution_;
  srv.request.manip_name = "arm";
  srv.request.joint_state.header.frame_id = root_name_;
  srv.request.joint_state.position = ik_seed_state;
  srv.request.pose_stamped.pose = ik_pose;
  srv.request.pose_stamped.header.frame_id = root_name_;
  srv.request.pose_stamped.header.stamp = ros::Time::now();
  srv.request.iktype = "translationdirection5d";
  srv.request.filteroptions = 0;
  ROS_DEBUG("Call OpenRave IK");
    ik_service_.call(srv);

    ROS_DEBUG("OpenRave Result %d", srv.response.error_code.val);

  if (srv.response.error_code.val == srv.response.error_code.SUCCESS)
  {

    if (srv.response.solutions.points.size() >= 1)
    {
      ROS_DEBUG("OpenRave IK found several solutions, we discard all despite the first one");
    }
    solution_.resize(dimension_);
    solution_ = srv.response.solutions.points[0].positions;
  }

  bool callback_check = true;
  if (solution_callback.empty())
    callback_check = false;

  if (srv.response.error_code.val == srv.response.error_code.SUCCESS)
  {
    if (callback_check)
    {
      jointSolutionCallback(solution_, ik_pose, error_code);
      if (error_code.val == error_code.SUCCESS)
      {
        solution.resize(dimension_);
        solution = solution_;
        error_code_int = 1; //SUCCESS
        ROS_DEBUG("return ture");
        return true;
      }
    }
    else
    {
      error_code.val = error_code.SUCCESS;
      solution.resize(dimension_);
      solution = solution_;
      error_code_int = 1; //SUCCESS
      ROS_DEBUG("return ture");
      return true;
    }
  }
  else
  {
    ROS_WARN("An IK solution could not be found");
    error_code_int = -2;
    ROS_WARN("return false");
    return false;
  }
  ROS_ERROR("searchPoistionIK run till the end, this should not happen");
  return false;
}

bool KatanaKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                           const std::vector<double> &joint_angles,
                                           std::vector<geometry_msgs::Pose> &poses)
{
  if (!active_)
  {
    ROS_ERROR("kinematics not active");
    return false;
  }

  kinematics_msgs::GetPositionFK srv;

  // TODO extrem unschoen oder?
  srv.request.header.frame_id = "katana_base_link";
  srv.request.fk_link_names = link_names;
  srv.request.robot_state.joint_state.name = link_names;
  srv.request.robot_state.joint_state.position = joint_angles;

  fk_service_.call(srv);

  poses.resize(link_names.size());

  if (srv.response.error_code.val == srv.response.error_code.NO_FK_SOLUTION)
  {
    ROS_ERROR("Could not find a FK");
    return false;
  }

  if (srv.response.error_code.val == srv.response.error_code.SUCCESS)
  {

    ROS_DEBUG("Successfully computed FK...");

    for (size_t i = 0; i < poses.size(); i++)
    {
      poses[i] = srv.response.pose_stamped[i].pose;
      ROS_DEBUG("Joint: %s Pose: %f %f %f // %f %f %f %f", link_names[i].c_str(), poses[i].position.x,
          poses[i].position.y,
          poses[i].position.z,
          poses[i].orientation.x,
          poses[i].orientation.y,
          poses[i].orientation.z,
          poses[i].orientation.w);
    }

    return true;
  }
  else
  {

    ROS_ERROR("Could not compute FK");

    return false;
  }
}
std::string KatanaKinematicsPlugin::getBaseFrame()
{
  if (!active_)
  {
    ROS_ERROR("kinematics not active");
    return std::string("");
  }
  return "katana_base_link";
}

std::string KatanaKinematicsPlugin::getToolFrame()
{
  if (!active_ || ik_solver_info_.link_names.empty())
  {
    ROS_ERROR("kinematics not active");
    return std::string("");
  }

  //ROS_ERROR("Giving Tool Frame: %s",ik_solver_info_.link_names[0].c_str());
  //return "katana_gripper_tool_frame";
  return ik_solver_info_.link_names[0];
}

std::vector<std::string> KatanaKinematicsPlugin::getJointNames()
{
  if (!active_)
  {
    std::vector<std::string> empty;
    ROS_ERROR("kinematics not active");
    return empty;
  }
  return ik_solver_info_.joint_names;
}

std::vector<std::string> KatanaKinematicsPlugin::getLinkNames()
{
  if (!active_)
  {
    std::vector<std::string> empty;
    ROS_ERROR("kinematics not active");
    return empty;
  }

  return fk_solver_info_.link_names;
}

bool KatanaKinematicsPlugin::getUrdfInfo(urdf::Model &robot_model, const std::string &root_name,
                                         const std::string &tip_name, kinematics_msgs::KinematicSolverInfo &chain_info)
{
  ROS_INFO("starting to read urdf...");
  boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
  boost::shared_ptr<const urdf::Joint> joint;
  while (link && link->name != root_name)
  {
    joint = robot_model.getJoint(link->parent_joint->name);
    ROS_INFO("Process %s",link->parent_joint->name.c_str());
    if (!joint)
    {
      ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
      return false;
    }
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      float lower, upper;
      int hasLimits;
      if (joint->type != urdf::Joint::CONTINUOUS)
      {
        lower = joint->limits->lower;
        upper = joint->limits->upper;
        hasLimits = 1;
      }
      else
      {
        lower = -M_PI;
        urdf::Model robot_model;
        upper = M_PI;
        hasLimits = 0;
      }
      chain_info.joint_names.push_back(joint->name);
      motion_planning_msgs::JointLimits limits;
      limits.joint_name = joint->name;
      limits.has_position_limits = hasLimits;
      limits.min_position = lower;
      limits.max_position = upper;
      chain_info.limits.push_back(limits);
    }
    link = robot_model.getLink(link->getParent()->name);
  }
  link = robot_model.getLink(tip_name);
  if (link)
    chain_info.link_names.push_back(tip_name);

  std::reverse(chain_info.limits.begin(), chain_info.limits.end());
  std::reverse(chain_info.joint_names.begin(), chain_info.joint_names.end());
  ROS_INFO("Got all infos out of URDF");
  return true;
}

}
// namespace
