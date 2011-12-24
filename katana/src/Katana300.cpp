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
 * Katana300.cpp
 *
 *  Created on: Dec 13, 2011
 *  Authors:
 *    Hannes Raudies <h.raudies@hs-mannheim.de>
 *    Martin Günther <mguenthe@uos.de>
 */

#include <katana/Katana300.h>

namespace katana
{

Katana300::Katana300() :
    Katana()
{
  desired_angles_ = getMotorAngles();
  setLimits();
}

Katana300::~Katana300()
{
}

void Katana300::setLimits()
{

  kni->setMotorAccelerationLimit(0, 1);
  kni->setMotorVelocityLimit(0, 30);

  for (size_t i = 1; i < NUM_MOTORS; i++)
  {
    // These two settings probably only influence KNI functions like moveRobotToEnc(),
    // openGripper() and so on, and not the spline trajectories. We still set them
    // just to be sure.
    kni->setMotorAccelerationLimit(i, 1);
    kni->setMotorVelocityLimit(i, 25);
  }

}

void Katana300::freezeRobot()
{
  boost::recursive_mutex::scoped_lock lock(kni_mutex);
  kni->freezeRobot();
}

bool Katana300::moveJoint(int jointIndex, double turningAngle)
{

  desired_angles_[jointIndex] = turningAngle;

  return Katana::moveJoint(jointIndex, turningAngle);
}

void Katana300::refreshMotorStatus()
{
  Katana::refreshEncoders();
  Katana::refreshMotorStatus();
}

bool Katana300::allJointsReady()
{
  std::vector<double> motor_angles = getMotorAngles();

  for (size_t i = 0; i < NUM_JOINTS; i++)
  {
    //Velocity 0 might be a problem
    if (motor_status_[i] == MSF_MOTCRASHED)
      return false;
    else if (fabs(desired_angles_[i] - motor_angles[i]) < 0.01)
    {
      if (motor_velocities_[i] != 0)
        return false;
    }
    else
      return false;
  }

  return true;
}

bool Katana300::allMotorsReady()
{
  std::vector<double> motor_angles = getMotorAngles();

  for (size_t i = 0; i < NUM_JOINTS; i++)
  {
    //Velocity 0 might be a problem
    if (motor_status_[i] == MSF_MOTCRASHED)
      return false;
    else if (fabs(desired_angles_[i] - motor_angles[i]) < 10)
    {
      if (motor_velocities_[i] != 0)
      {
        return false;
      }
    }
    else
      return false;
  }

  return true;
}

}
