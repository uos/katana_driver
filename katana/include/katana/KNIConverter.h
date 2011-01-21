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
 * KNIConverter.h
 *
 *  Created on: 21.01.2011
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#ifndef KNICONVERTER_H_
#define KNICONVERTER_H_

#include <ros/ros.h>
#include <KNI/kmlFactories.h>

namespace katana
{

class KNIConverter
{
public:
  KNIConverter(std::string config_file_path);
  virtual ~KNIConverter();

  short angle_rad2enc(int index, double angle);
  double angle_enc2rad(int index, int encoders);

  short vel_rad2enc(int index, double vel);
  short acc_rad2enc(int index, double acc);
  short jerk_rad2enc(int index, double jerk);

  double vel_enc2rad(int index, short encoders);
  double acc_enc2rad(int index, short encoders);
  double jerk_enc2rad(int index, short encoders);

private:
  /// KNI time is in 10 milliseconds (most of the time), ROS time is in seconds
  static const double KNI_TO_ROS_TIME = 100.0;
  static const size_t NUM_MOTORS = 6;

  KNI::kmlFactory config_;

  short vel_acc_jerk_rad2enc(int index, double vel_acc_jerk);
  double vel_acc_jerk_enc2rad(int index, short encoders);

  double deg2rad(const double deg);

};

/**
 * constants for converting between the KNI gripper angle and the URDF gripper angle
 */
static const double KNI_GRIPPER_CLOSED_ANGLE = 0.21652991032554647;
static const double KNI_GRIPPER_OPEN_ANGLE = -2.0047969889958925;

static const double URDF_GRIPPER_CLOSED_ANGLE = -0.4; /// should be equal to the value in the urdf description
static const double URDF_GRIPPER_OPEN_ANGLE = 0.4; /// should be equal to the value in the urdf description

static const double KNI_TO_URDF_GRIPPER_FACTOR = (URDF_GRIPPER_OPEN_ANGLE - URDF_GRIPPER_CLOSED_ANGLE)
    / (KNI_GRIPPER_OPEN_ANGLE - KNI_GRIPPER_CLOSED_ANGLE);


}

#endif /* KNICONVERTER_H_ */
