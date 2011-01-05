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
 * JointStatePublisher.cpp
 *
 *  Created on: 06.12.2010
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#include "katana/JointStatePublisher.h"

namespace katana
{

JointStatePublisher::JointStatePublisher(ros::NodeHandle n, boost::shared_ptr<AbstractKatana> katana) :
  nh(n), katana(katana), pub(nh.advertise<sensor_msgs::JointState> ("joint_states", 1000))
{
}

JointStatePublisher::~JointStatePublisher()
{
}

void JointStatePublisher::loopOnce()
{
  /* ************** Publish joint angles ************** */
  sensor_msgs::JointStatePtr jsMsg = boost::make_shared<sensor_msgs::JointState>();
  std::vector<double> motorAngles = katana->getMotorAngles();

  // TODO: read joint names from katana, for the fingers use dependent_joints like the joint_state_publisher does

  jsMsg->name.push_back("katana_motor1_pan_joint");
  jsMsg->position.push_back(motorAngles[0]);
  jsMsg->name.push_back("katana_motor2_lift_joint");
  jsMsg->position.push_back(motorAngles[1]);
  jsMsg->name.push_back("katana_motor3_lift_joint");
  jsMsg->position.push_back(motorAngles[2]);
  jsMsg->name.push_back("katana_motor4_lift_joint");
  jsMsg->position.push_back(motorAngles[3]);
  jsMsg->name.push_back("katana_motor5_wrist_roll_joint");
  jsMsg->position.push_back(motorAngles[4]);

  // TODO: call getGripperAngle or something
  jsMsg->name.push_back("katana_r_finger_joint");
  jsMsg->position.push_back(motorAngles[5]);
  jsMsg->name.push_back("katana_l_finger_joint");
  jsMsg->position.push_back(motorAngles[5]); // both right and left finger are controlled by motor 6

  jsMsg->header.stamp = ros::Time::now();
  pub.publish(jsMsg); // NOTE: jsMsg must not be changed after publishing; use reset() if necessary (http://www.ros.org/wiki/roscpp/Internals)
}


}
