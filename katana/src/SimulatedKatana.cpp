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
 * SimulatedKatana.cpp
 *
 *  Created on: 20.12.2010
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#include "../include/katana/SimulatedKatana.h"

namespace katana
{

SimulatedKatana::SimulatedKatana(ros::NodeHandle n) :
  AbstractKatana(n)
{
  // Creates a "hold current position" trajectory.
  boost::shared_ptr<SpecifiedTrajectory> hold_ptr(new SpecifiedTrajectory(1));
  SpecifiedTrajectory &hold = *hold_ptr;
  hold[0].start_time = ros::Time::now().toSec() - 0.001;
  hold[0].duration = 0.0;
  hold[0].splines.resize(NUM_JOINTS);

  hold[0].splines[0].coef[0] = angle_rad2enc(0, 0.12);
  hold[0].splines[1].coef[0] = angle_rad2enc(1, 2.163);
  hold[0].splines[2].coef[0] = angle_rad2enc(2, 0.935);
  hold[0].splines[3].coef[0] = angle_rad2enc(3, 1.116);
  hold[0].splines[4].coef[0] = angle_rad2enc(4, 0.152);

  current_trajectory_ = hold_ptr;
}

SimulatedKatana::~SimulatedKatana()
{

}

void SimulatedKatana::refreshEncoders()
{
  const SpecifiedTrajectory &traj = *current_trajectory_;

  // Determines which segment of the trajectory to use
  size_t seg = 0;
  while (seg + 1 < traj.size() && traj[seg + 1].start_time <= ros::Time::now().toSec())
  {
    seg++;
  }

  for (size_t j = 0; j < NUM_JOINTS; j++)
  {
    double pos_t, vel_t, acc_t;
    sampleSplineWithTimeBounds(traj[seg].splines[j].coef, traj[seg].duration,
                               ros::Time::now().toSec() - traj[seg].start_time, pos_t, vel_t, acc_t);

    motor_angles_[j] = angle_enc2rad(j, pos_t);
    motor_velocities_[j] = angle_enc2rad(j, vel_t);
  }
}

std::vector<double> SimulatedKatana::getCoordinates()
{
  // TODO later: real implementation
  return std::vector<double>(6, 0.0);
}

bool SimulatedKatana::executeTrajectory(boost::shared_ptr<SpecifiedTrajectory> traj_ptr, ros::Time start_time)
{
  // ------- wait until start time
  ros::Time::sleepUntil(start_time);

  // TODO: I modify traj now, copy first because it's shared, make const

  SpecifiedTrajectory &traj = *traj_ptr;

  // ------- fix start time
  assert(traj[0].start_time == 0.0);

  for (size_t i = 0; i < traj.size(); ++i) {
    traj[i].start_time += start_time.toSec();
  }

  current_trajectory_ = traj_ptr;
  return true;
}

int SimulatedKatana::angle_rad2enc(int index, double angle)
{
  return SIM_ENC_PER_RAD * angle;
}

double SimulatedKatana::angle_enc2rad(int index, int encoders)
{
  return (double)encoders / (double)SIM_ENC_PER_RAD;
}

int SimulatedKatana::velocity_rad2enc(int index, double angular_velocity)
{
  return SIM_ENC_PER_RAD * angular_velocity;
}

double SimulatedKatana::velocity_enc2rad(int index, int encoder_velocity)
{
  return (double)encoder_velocity / (double)SIM_ENC_PER_RAD;
}

}
