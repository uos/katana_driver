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
 * Katana.h
 *
 *  Created on: 06.12.2010
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#ifndef KATANA_H_
#define KATANA_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread/recursive_mutex.hpp>

#include <kniBase.h>

#include <katana/SpecifiedTrajectory.h>
#include <katana/EulerTransformationMatrices.hh>
#include <katana/AbstractKatana.h>

namespace katana
{

/**
 * @brief Wrapper class around the KNI (Katana Native Library).
 *
 * All access to the Katana hardware should happen through this class. There must
 * always be only one instance of this class. This class should be thread-safe.
 */
class Katana : public AbstractKatana
{
public:
  Katana(ros::NodeHandle n);
  virtual ~Katana();

  void refreshEncoders();
  std::vector<double> getCoordinates();
  bool executeTrajectory(boost::shared_ptr<SpecifiedTrajectory> traj, ros::Time start_time);

  int angle_rad2enc(int index, double angle);
  double angle_enc2rad(int index, int encoders);
  int velocity_rad2enc(int index, double angular_velocity);
  double velocity_enc2rad(int index, int encoder_velocity);

private:
  boost::shared_ptr<CLMBase> kni;
  CCplSerialCRC* protocol;
  CCdlSocket* device;

  boost::recursive_mutex kni_mutex;

  std::vector<TMotStsFlg> motor_status_;

  void calibrate();
  bool someMotorCrashed();
  bool allJointsReady();
};

}

#endif /* KATANA_H_ */
