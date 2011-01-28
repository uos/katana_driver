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

SimulatedKatana::SimulatedKatana() :
  AbstractKatana()
{
  // Creates a "hold current position" trajectory.
  boost::shared_ptr<SpecifiedTrajectory> hold_ptr(new SpecifiedTrajectory(1));
  SpecifiedTrajectory &hold = *hold_ptr;
  hold[0].start_time = ros::Time::now().toSec() - 0.001;
  hold[0].duration = 0.0;
  hold[0].splines.resize(NUM_JOINTS);

  hold[0].splines[0].coef[0] = -3.022;
  hold[0].splines[1].coef[0] = 2.163;
  hold[0].splines[2].coef[0] = -2.207;
  hold[0].splines[3].coef[0] = -2.026;
  hold[0].splines[4].coef[0] = -2.990;

  current_trajectory_ = hold_ptr;

  motor_angles_[5] = 0.30;
  motor_velocities_[5] = 0.0;
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
    sampleSplineWithTimeBounds(traj[seg].splines[j].coef, traj[seg].duration, ros::Time::now().toSec()
        - traj[seg].start_time, pos_t, vel_t, acc_t);

    motor_angles_[j] = pos_t;
    motor_velocities_[j] = vel_t;
  }
}

bool SimulatedKatana::executeTrajectory(boost::shared_ptr<SpecifiedTrajectory> traj_ptr)
{
  // ------- wait until start time
  ros::Time::sleepUntil(ros::Time(traj_ptr->at(0).start_time));

  current_trajectory_ = traj_ptr;
  return true;
}

}
