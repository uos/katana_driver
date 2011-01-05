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
 * AbstractKatana.h
 *
 *  Created on: 20.12.2010
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#ifndef ABSTRACTKATANA_H_
#define ABSTRACTKATANA_H_

#include <ros/ros.h>
#include <urdf/joint.h>

#include <katana/SpecifiedTrajectory.h>

namespace katana
{

const size_t NUM_MOTORS = 6;
const size_t NUM_JOINTS = NUM_MOTORS - 1;

class AbstractKatana
{
public:
  AbstractKatana(ros::NodeHandle n);
  virtual ~AbstractKatana();

  virtual void refreshEncoders() = 0;
  virtual std::vector<double> getCoordinates() = 0;
  virtual bool executeTrajectory(boost::shared_ptr<SpecifiedTrajectory> traj, ros::Time start_time) = 0;

  virtual int getJointIndex(std::string joint_name);
  virtual std::vector<std::string> getJointNames();
  virtual std::vector<int> getJointTypes();

  virtual std::vector<double> getMotorAngles();
  virtual std::vector<double> getMotorVelocities();

  virtual int angle_rad2enc(int index, double angle) = 0;
  virtual double angle_enc2rad(int index, int encoders) = 0;
  virtual int velocity_rad2enc(int index, double angular_velocity) = 0;
  virtual double velocity_enc2rad(int index, int encoder_velocity) = 0;

  virtual double ros2kni_time(ros::Time ros_time);
  virtual double ros2kni_time(ros::Duration ros_time);
  virtual ros::Time kni2ros_time(double kni_time);

protected:
  ros::NodeHandle nh;

  // only the 5 "real" joints:
  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;

  // all motors (the 5 "real" joints plus the gripper)
  std::vector<double> motor_angles_;
  std::vector<double> motor_velocities_;
};

}

#endif /* ABSTRACTKATANA_H_ */
