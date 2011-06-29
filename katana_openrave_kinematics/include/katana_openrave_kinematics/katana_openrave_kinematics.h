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
 *
 * katana_kinematics_plugin.cpp
 * based on the pr2_arm_kinematics_plugin.h by Sachin Chitta
 *
 *  Created on: 30.05.2010
 *  Author: Henning Deeken  // hdeeken@uos.de
 */
#ifndef KATANA_OPENRAVE_KINEMATICS_H
#define KATANA_OPENRAVE_KINEMATICS_H

#include <algorithm>
#include <numeric>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <motion_planning_msgs/ArmNavigationErrorCodes.h>
#include <orrosplanning/IK.h>
#include <urdf/model.h>
#include <kinematics_base/kinematics_base.h>
#include <arm_kinematics_constraint_aware/arm_kinematics_constraint_aware_utils.h>
#include <ompl/util/RandomNumbers.h>

namespace katana_openrave_kinematics
{
class KatanaKinematicsPlugin : public kinematics::KinematicsBase
{
public:

  /** @class
   *  @brief Plugin-able interface to the Katana arm kinematics
   */
  KatanaKinematicsPlugin();

  /**
   *  @brief Specifies if the node is active or not
   *  @return True if the node is active, false otherwise.
   */
  bool isActive();

  /**
   * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
   * @param ik_link_name - the name of the link for which IK is being computed
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @return True if a valid solution was found, false otherwise
   */
  bool getPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state,
                     std::vector<double> &solution, int &error_code);

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @return True if a valid solution was found, false otherwise
   */

  bool searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state,
                        const double &timeout, std::vector<double> &solution, int &error_code);
  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state,
                        const double &timeout, std::vector<double> &solution, const boost::function<
                            void(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_solution,
                                 int &error_code)> &desired_pose_callback, const boost::function<
                            void(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_solution,
                                 int &error_code)> &solution_callback, int &error_code);

  /**
   * @brief Given a set of joint angles and a set of links, compute their pose
   * @param request  - the request contains the joint angles, set of links for which poses are to be computed and a timeout
   * @param response - the response contains stamped pose information for all the requested links
   * @return True if a valid solution was found, false otherwise
   */
  bool getPositionFK(const std::vector<std::string> &link_names, const std::vector<double> &joint_angles, std::vector<
      geometry_msgs::Pose> &poses);

  /**
   * @brief  Initialization function for the kinematics
   * @return True if initialization was successful, false otherwise
   */
  bool initialize(std::string name);

  /**
   * @brief  Return the frame in which the kinematics is operating
   * @return the string name of the frame in which the kinematics is operating
   */
  std::string getBaseFrame();

  /**
   * @brief  Return the links for which kinematics can be computed
   */
  std::string getToolFrame();

  /**
   * @brief  Return all the joint names in the order they are used internally
   */
  std::vector<std::string> getJointNames();

  /**
   * @brief  Return all the link names in the order they are represented internally
   */
  std::vector<std::string> getLinkNames();

protected:

  bool active_;
  urdf::Model robot_model_;

  ros::NodeHandle node_handle_, root_handle_;

  ros::ServiceClient ik_service_, fk_service_, ik_solver_info_service_, fk_solver_info_service_;
  tf::TransformListener tf_;
  std::string root_name_;
  int dimension_;

  kinematics_msgs::KinematicSolverInfo ik_solver_info_, fk_solver_info_;

  boost::function<void(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_solution, int &error_code)>
                                                                                                                     desiredPoseCallback_;
  boost::function<void(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_solution, int &error_code)>
                                                                                                                     solutionCallback_;
  void desiredPoseCallback(const std::vector<double>& joint_angles, const geometry_msgs::Pose& ik_pose,
                           motion_planning_msgs::ArmNavigationErrorCodes& error_code);

  void jointSolutionCallback(const std::vector<double>& joint_angles,
                                                     const geometry_msgs::Pose& ik_pose,
                                                     motion_planning_msgs::ArmNavigationErrorCodes& error_code);

};
}

#endif
