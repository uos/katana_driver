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
  // TODO: constants

  // TODO: setting the limits this low shouldn't be necessary; the limits should
  //       be set to 2 (acc.) and 180 (vel.) and tested on real Katana 300


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

/**
 * The Katana 300 moves to a weird configuration whenever flushMoveBuffers() is called.
 * That's why we override freezeRobot() and skip this call.
 */
void Katana300::freezeRobot()
{
  boost::recursive_mutex::scoped_lock lock(kni_mutex);
  kni->freezeRobot();
}

/**
 * Override to store desired_angles_.
 */
bool Katana300::moveJoint(int jointIndex, double turningAngle, double maxTurningSpeed)
{

  desired_angles_[jointIndex] = turningAngle;

  return Katana::moveJoint(jointIndex, turningAngle, maxTurningSpeed);
}

/**
 * We have to call refreshEncoders() because we're using the destination position
 * instead of the motor status flags in allJointsReady/allMotorsReady.
 */
void Katana300::refreshMotorStatus()
{
  Katana::refreshEncoders();
  Katana::refreshMotorStatus();
}

/**
 * The Katana 300 never returns MSF_DESPOS or MSF_NLINMOV, so we have to check
 * manually whether the arm stopped at the target position.
 */
bool Katana300::allJointsReady()
{
  std::vector<double> motor_angles = getMotorAngles();

  for (size_t i = 0; i < NUM_JOINTS; i++)
  {
    if (motor_status_[i] == MSF_MOTCRASHED)
      return false;
    if (fabs(desired_angles_[i] - motor_angles[i]) > JOINTS_STOPPED_POS_TOLERANCE)
      return false;
    if (fabs(motor_velocities_[i]) > JOINTS_STOPPED_VEL_TOLERANCE)
      return false;
  }

  return true;
}

/**
 * The Katana 300 never returns MSF_DESPOS or MSF_NLINMOV, so we have to check
 * manually whether the arm stopped at the target position.
 */
bool Katana300::allMotorsReady()
{
  std::vector<double> motor_angles = getMotorAngles();

  for (size_t i = 0; i < NUM_MOTORS; i++)
  {
    if (motor_status_[i] == MSF_MOTCRASHED)
      return false;
    if (fabs(desired_angles_[i] - motor_angles[i]) > JOINTS_STOPPED_POS_TOLERANCE)
      return false;
    if (fabs(motor_velocities_[i]) > JOINTS_STOPPED_VEL_TOLERANCE)
      return false;
  }

  return true;
}

void Katana300::testSpeed()
{
  ros::Rate idleWait(5);
  std::vector<double> pos1_angles(NUM_MOTORS);
  std::vector<double> pos2_angles(NUM_MOTORS);

  // these are safe values, i.e., no self-collision is possible
  pos1_angles[0] = 2.88;
  pos2_angles[0] = -3.02;

  pos1_angles[1] = 0.15;
  pos2_angles[1] = 2.16;

  pos1_angles[2] = 1.40;
  pos2_angles[2] = -2.21;

  pos1_angles[3] = 0.50;
  pos2_angles[3] = -2.02;

  pos1_angles[4] = 2.86;
  pos2_angles[4] = -2.98;

  pos1_angles[5] = -0.44;
  pos2_angles[5] = 0.30;

  for (size_t i = 0; i < NUM_MOTORS; i++)
  {
    int pos1_encoders = (int)converter->angle_rad2enc(i, pos1_angles[i]);
    int pos2_encoders = (int)converter->angle_rad2enc(i, pos2_angles[i]);

    int accel = kni->getMotorAccelerationLimit(i);
    int max_vel = kni->getMotorVelocityLimit(i);

    ROS_INFO("Motor %zu - acceleration: %d (= %f), max speed: %d (=%f)", i, accel, 2.0 * converter->acc_enc2rad(i, accel), max_vel, converter->vel_enc2rad(i, max_vel));
    ROS_INFO("KNI encoders: %d, %d", kni->GetBase()->GetMOT()->arr[i].GetEncoderMinPos(), kni->GetBase()->GetMOT()->arr[i].GetEncoderMaxPos());
    ROS_INFO("moving to encoders: %d, %d", pos1_encoders, pos2_encoders);
    ROS_INFO("current encoders: %d", kni->getMotorEncoders(i, true));

    ROS_INFO("Moving to min");
    {
      boost::recursive_mutex::scoped_lock lock(kni_mutex);
      kni->moveMotorToEnc(i, pos1_encoders, true, 50, 60000);
    }

//      do
//      {
//        idleWait.sleep();
//        refreshMotorStatus();
//
//      } while (!allMotorsReady());

    ROS_INFO("Moving to max");
    {
      boost::recursive_mutex::scoped_lock lock(kni_mutex);
      kni->moveMotorToEnc(i, pos2_encoders, true, 50, 60000);
    }

//      do
//      {
//        idleWait.sleep();
//        refreshMotorStatus();
//      } while (!allMotorsReady());
  }

  // Result:
  //  Motor 0 - acceleration: 2 (= -4.908739), max speed: 180 (=-2.208932)
  //  Motor 1 - acceleration: 2 (= -2.646220), max speed: 180 (=-1.190799)
  //  Motor 2 - acceleration: 2 (= 5.292440), max speed: 180 (=2.381598)
  //     --> wrong! the measured values are more like 2.6, 1.2
  //
  //  Motor 3 - acceleration: 2 (= -4.908739), max speed: 180 (=-2.208932)
  //  Motor 4 - acceleration: 2 (= -4.908739), max speed: 180 (=-2.208932)
  //  Motor 5 - acceleration: 2 (= 1.597410), max speed: 180 (=0.718834)
  //     (TODO: the gripper duration can be calculated from this)
}


}
