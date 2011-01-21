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

#include <katana/KNIKinematics.h>

namespace katana
{

KNIKinematics::KNIKinematics()
{
  joint_names_.resize(NUM_JOINTS);


  // ------- get parameters
  ros::NodeHandle pn("~");

  std::string config_file_path;

  ros::param::param("~/katana/config_file_path", config_file_path, ros::package::getPath("kni")
      + "/KNI_4.3.0/configfiles450/katana6M90A_G.cfg");

  converter_ = new KNIConverter(config_file_path);

  XmlRpc::XmlRpcValue joint_names;

  // Gets all of the joints
  if (!pn.getParam("joints", joint_names))
  {
    ROS_ERROR("No joints given. (namespace: %s)", pn.getNamespace().c_str());
  }
  if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Malformed joint specification.  (namespace: %s)", pn.getNamespace().c_str());
  }
  if (joint_names.size() != (size_t)NUM_JOINTS)
  {
    ROS_ERROR("Wrong number of joints! was: %zu, expected: %zu", joint_names.size(), NUM_JOINTS);
  }
  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    XmlRpc::XmlRpcValue &name_value = joint_names[i];
    if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("Array of joint names should contain all strings.  (namespace: %s)",
          pn.getNamespace().c_str());
    }

    joint_names_[i] = (std::string)name_value;
  }

  get_kinematic_solver_info_server_ = nh_.advertiseService("get_kinematic_solver_info",
                                                           &KNIKinematics::get_kinematic_solver_info, this);

  // TODO: register GetPositionFK, GetPositionIK

}

KNIKinematics::~KNIKinematics()
{
  delete converter_;
}

bool KNIKinematics::get_kinematic_solver_info(kinematics_msgs::GetKinematicSolverInfo::Request &req,
                                              kinematics_msgs::GetKinematicSolverInfo::Response &res)
{
  res.kinematic_solver_info.joint_names = joint_names_;
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

  pose = getCoordinates(jointAngles);

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


/**
 * Return the position of the tool center point as calculated by the KNI. Uses the current position as input.
 *
 * @return a vector <x, y, z, r, p, y>; xyz in [m], rpy in [rad]
 */
std::vector<double> KNIKinematics::getCoordinates()
{
  const double KNI_TO_ROS_LENGTH = 0.001; // the conversion factor from KNI coordinates (in mm) to ROS coordinates (in m)

  double kni_z, kni_x1, kni_z2;
  std::vector<double> pose(6, 0.0);

  ikBase_.getCoordinates(pose[0], pose[1], pose[2], kni_z, kni_x1, kni_z2, false);

  // zyx = yaw, pitch, roll = pose[5], pose[4], pose[3]
  EulerTransformationMatrices::zxz_to_zyx_angles(kni_z, kni_x1, kni_z2, pose[5], pose[4], pose[3]);

  pose[0] = pose[0] * KNI_TO_ROS_LENGTH;
  pose[1] = pose[1] * KNI_TO_ROS_LENGTH;
  pose[2] = pose[2] * KNI_TO_ROS_LENGTH;

  return pose;
}

/**
 * Return the position of the tool center point as calculated by the KNI.
 *
 * @param jointAngles the joint angles to compute the pose for (direct kinematics)
 * @return a vector <x, y, z, r, p, y>; xyz in [m], rpy in [rad]
 */
std::vector<double> KNIKinematics::getCoordinates(std::vector<double> jointAngles)
{
  const double KNI_TO_ROS_LENGTH = 0.001; // the conversion factor from KNI coordinates (in mm) to ROS coordinates (in m)

  std::vector<double> result(6, 0.0);
  std::vector<double> pose(6, 0.0);
  std::vector<int> encoders;

  for (size_t i = 0; i < jointAngles.size(); i++)
    encoders.push_back(converter_->angle_rad2enc(i, jointAngles[i]));

  ikBase_.getCoordinatesFromEncoders(pose, encoders);

  // zyx = yaw, pitch, roll = result[5], result[4], result[3]
  EulerTransformationMatrices::zxz_to_zyx_angles(pose[3], pose[4], pose[5], result[5], result[4], result[3]);

  result[0] = pose[0] * KNI_TO_ROS_LENGTH;
  result[1] = pose[1] * KNI_TO_ROS_LENGTH;
  result[2] = pose[2] * KNI_TO_ROS_LENGTH;

  return result;
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "katana_arm_kinematics");

  katana::KNIKinematics node;

  ros::spin();
  return 0;
}
