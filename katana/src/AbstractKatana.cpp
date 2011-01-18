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
 * AbstractKatana.cpp
 *
 *  Created on: 20.12.2010
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#include <katana/AbstractKatana.h>

namespace katana
{

AbstractKatana::AbstractKatana(ros::NodeHandle n)
{
  // names and types: only the 5 "real" joints
  joint_names_.resize(NUM_JOINTS);
  joint_types_.resize(NUM_JOINTS);

  // angles and velocities: the 5 "real" joints + gripper
  motor_angles_.resize(NUM_MOTORS);
  motor_velocities_.resize(NUM_MOTORS);

  /* ********* get parameters ********* */
  ros::NodeHandle pn("~");

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
    joint_types_[i] = urdf::Joint::REVOLUTE; // all of our joints are of type revolute
  }
}

AbstractKatana::~AbstractKatana()
{
}

void AbstractKatana::freezeRobot()
{
  // do nothing (can be overridden)
}

/* ******************************** joints + motors ******************************** */

int AbstractKatana::getJointIndex(std::string joint_name)
{
  for (int i = 0; i < (int)joint_names_.size(); i++)
  {
    if (joint_names_[i] == joint_name)
      return i;
  }

  ROS_ERROR("Joint not found: %s.", joint_name.c_str());
  return -1;
}

std::vector<std::string> AbstractKatana::getJointNames()
{
  return joint_names_;
}

std::vector<int> AbstractKatana::getJointTypes()
{
  return joint_types_;
}

std::vector<double> AbstractKatana::getMotorAngles()
{
  return motor_angles_;
}

std::vector<double> AbstractKatana::getMotorVelocities()
{
  return motor_velocities_;
}

}
