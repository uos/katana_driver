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
 * SimulatedKatana.h
 *
 *  Created on: 20.12.2010
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#ifndef SIMULATEDKATANA_H_
#define SIMULATEDKATANA_H_

#include "AbstractKatana.h"
#include "spline_functions.h"

namespace katana
{
const int SIM_ENC_PER_RAD = 10000;

class SimulatedKatana : public katana::AbstractKatana
{
public:
  SimulatedKatana(ros::NodeHandle n);
  virtual ~SimulatedKatana();

  virtual void refreshEncoders();
  virtual std::vector<double> getCoordinates();
  virtual bool executeTrajectory(boost::shared_ptr<SpecifiedTrajectory> traj, ros::Time start_time);

  virtual int angle_rad2enc(int index, double angle);
  virtual double angle_enc2rad(int index, int encoders);
  virtual int velocity_rad2enc(int index, double angular_velocity);
  virtual double velocity_enc2rad(int index, int encoder_velocity);

private:
  boost::shared_ptr<SpecifiedTrajectory> current_trajectory_;
};

}

#endif /* SIMULATEDKATANA_H_ */
