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
 * Katana.cpp
 *
 *  Created on: 06.12.2010
 *      Author: Martin Günther <mguenthe@uos.de>
 *
 */

#include "katana/Katana.h"

namespace katana
{

Katana::Katana() :
  AbstractKatana()
{
  ros::NodeHandle pn("~");
  motor_status_.resize(NUM_MOTORS);

  /* ********* get parameters ********* */
  std::string ip, config_file_path;
  int port;

  pn.param<std::string> ("ip", ip, "192.168.1.1");
  pn.param("port", port, 5566);
  pn.param("config_file_path", config_file_path, ros::package::getPath("kni")
      + "/KNI_4.3.0/configfiles450/katana6M90A_G.cfg");

  converter = new KNIConverter(config_file_path);

  try
  {
    /* ********* open device: a network transport is opened in this case ********* */
    char* nonconst_ip = strdup(ip.c_str());
    device = new CCdlSocket(nonconst_ip, port);
    free(nonconst_ip);
    ROS_INFO("success:  port %d open", port);

    /* ********* init protocol ********* */
    protocol = new CCplSerialCRC();
    ROS_INFO("success: protocol class instantiated");

    protocol->init(device); //fails if no response from Katana
    ROS_INFO("success: communication with Katana initialized");

    /* ********* init robot ********* */
    kni.reset(new CLMBase());
    kni->create(config_file_path.c_str(), protocol);
    ROS_INFO("success: katana initialized");
  }
  catch (Exception &e)
  {
    ROS_ERROR("Exception during initialization: '%s'", e.message().c_str());
    return;
  }

  for (size_t i = 0; i < NUM_MOTORS; i++)
  {
    // These two settings probably only influence KNI functions like moveRobotToEnc(),
    // openGripper() and so on, and not the spline trajectories. We still set them
    // just to be sure.
    kni->setMotorAccelerationLimit(i, KNI_MAX_ACCELERATION);
    kni->setMotorVelocityLimit(i, KNI_MAX_VELOCITY);
  }

  /* ********* calibrate ********* */
  calibrate();
  ROS_INFO("success: katana calibrated");

  refreshEncoders();

  // boost::thread worker_thread(&Katana::test_speed, this);
}

Katana::~Katana()
{
  // protocol and device are members of kni, so we should be
  // the last ones using it before deleting them
  assert(kni.use_count() == 1);

  kni.reset(); // delete kni, so protocol and device won't be used any more
  delete protocol;
  delete device;
  delete converter;
}

void Katana::refreshEncoders()
{
  try
  {
    boost::recursive_mutex::scoped_lock lock(kni_mutex);
    CMotBase* motors = kni->GetBase()->GetMOT()->arr;

    kni->GetBase()->recvMPS(); // refresh all pvp->pos at once

    // Using recvMPS() instead of recvPVP() makes our updates 6 times
    // faster, since we only need one KNI call instead of 6. The KNI
    // needs exactly 40 ms for every update call.

    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
      // motors[i].recvPVP(); // refresh pvp->pos, pvp->vel and pvp->msf for single motor; throws ParameterReadingException
      const TMotPVP* pvp = motors[i].GetPVP();

      double current_angle = converter->angle_enc2rad(i, pvp->pos);
      double time_since_update = (ros::Time::now() - last_encoder_update_).toSec();

      if (last_encoder_update_ == ros::Time(0.0) || time_since_update == 0.0)
      {
        motor_velocities_[i] = 0.0;
      }
      else
      {
        motor_velocities_[i] = (current_angle - motor_angles_[i]) / time_since_update;
      }
      //  // only necessary when using recvPVP():
      //  motor_velocities_[i] = vel_enc2rad(i, pvp->vel) * (-1);  // the -1 is because the KNI is inconsistent about velocities

      motor_angles_[i] = current_angle;
    }

    //  // This is an ugly workaround, but apparently the velocities returned by the
    //  // Katana are wrong by a factor of exactly 0.5 for motor 2 and a factor of -1
    //  // for motor 4. This is only necessary when using recvPVP() to receive the
    //  // velocities directly from the KNI.
    //  motor_velocities_[2] *= 0.5;
    //  motor_velocities_[4] *= -1.0;

    last_encoder_update_ = ros::Time::now();
  }
  catch (WrongCRCException e)
  {
    ROS_ERROR("WrongCRCException: Two threads tried to access the KNI at once. This means that the locking in the Katana node is broken. (exception in refreshEncoders(): %s)", e.message().c_str());
  }
  catch (ReadNotCompleteException e)
  {
    ROS_ERROR("ReadNotCompleteException: Another program accessed the KNI. Please stop it and restart the Katana node. (exception in refreshEncoders(): %s)", e.message().c_str());
  }
  catch (ParameterReadingException e)
  {
    ROS_ERROR("ParameterReadingException: Could not receive PVP (Position Velocity PWM) parameters from a motor (exception in refreshEncoders(): %s)", e.message().c_str());
  }
  catch (Exception e)
  {
    ROS_ERROR("Unhandled exception in refreshEncoders(): %s", e.message().c_str());
  }
  catch (...)
  {
    ROS_ERROR("Unhandled exception in refreshEncoders()");
  }
}

void Katana::refreshMotorStatus()
{
  try
  {
    boost::recursive_mutex::scoped_lock lock(kni_mutex);
    CMotBase* motors = kni->GetBase()->GetMOT()->arr;

    kni->GetBase()->recvGMS(); // refresh all pvp->msf at once

    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
      const TMotPVP* pvp = motors[i].GetPVP();

      motor_status_[i] = pvp->msf;
      //  MSF_MECHSTOP    = 1,    //!< mechanical stopper reached, new: unused (default value)
      //  MSF_MAXPOS      = 2,    //!< max. position was reached, new: unused
      //  MSF_MINPOS      = 4,    //!< min. position was reached, new: calibrating
      //  MSF_DESPOS      = 8,    //!< in desired position, new: fixed, state holding
      //  MSF_NORMOPSTAT  = 16,   //!< trying to follow target, new: moving (polymb not full)
      //  MSF_MOTCRASHED  = 40,   //!< motor has crashed, new: collision (MG: this means that the motor has reached a collision limit (e.g., after executing a invalid spline) and has to be reset using unBlock())
      //  MSF_NLINMOV     = 88,   //!< non-linear movement ended, new: poly move finished
      //  MSF_LINMOV      = 152,  //!< linear movement ended, new: moving poly, polymb full
      //  MSF_NOTVALID    = 128   //!< motor data not valid

      /*
       *  Das Handbuch zu Katana4D sagt (zu ReadAxisState):
       *    0 = off (ausgeschaltet)
       *    8 = in gewünschter Position (Roboter fixiert)
       *   24 = im "normalen Bewegungsstatus"; versucht die gewünschte Position zu erreichen
       *   40 = wegen einer Kollision gestoppt
       *   88 = die Linearbewegung ist beendet
       *  128 = ungültige Achsendaten (interne Kommunikationsprobleme)
       */
    }
  }
  catch (WrongCRCException e)
  {
    ROS_ERROR("WrongCRCException: Two threads tried to access the KNI at once. This means that the locking in the Katana node is broken. (exception in refreshMotorStatus(): %s)", e.message().c_str());
  }
  catch (ReadNotCompleteException e)
  {
    ROS_ERROR("ReadNotCompleteException: Another program accessed the KNI. Please stop it and restart the Katana node. (exception in refreshMotorStatus(): %s)", e.message().c_str());
  }
  catch (Exception e)
  {
    ROS_ERROR("Unhandled exception in refreshMotorStatus(): %s", e.message().c_str());
  }
  catch (...)
  {
    ROS_ERROR("Unhandled exception in refreshMotorStatus()");
  }
}

/**
 * Sends the spline parameters to the Katana.
 *
 * @param traj
 */
bool Katana::executeTrajectory(boost::shared_ptr<SpecifiedTrajectory> traj)
{
  assert(traj->size() > 0);

  try
  {
    refreshMotorStatus();
    ROS_DEBUG("Motor status: %d, %d, %d, %d, %d, %d", motor_status_[0], motor_status_[1], motor_status_[2], motor_status_[3], motor_status_[4], motor_status_[5]);

    // ------- check if motors are blocked
    if (someMotorCrashed())
    {
      ROS_WARN("Motors are crashed before executing trajectory! Unblocking...");

      boost::recursive_mutex::scoped_lock lock(kni_mutex);
      kni->unBlock();
    }

    // ------- wait until all motors idle
    ros::Rate idleWait(10);
    while (!allJointsReady())
    {
      idleWait.sleep();
      refreshMotorStatus();
    }

    //// ------- move to start position
    //{
    //  assert(traj->at(0).splines.size() == NUM_JOINTS);
    //
    //  boost::recursive_mutex::scoped_lock lock(kni_mutex);
    //  std::vector<int> encoders;
    //
    //  for (size_t i = 0; i < NUM_JOINTS; i++) {
    //    encoders.push_back(converter->angle_rad2enc(i, traj->at(0).splines[i].coef[0]));
    //  }
    //
    //  std::vector<int> current_encoders = kni->getRobotEncoders(true);
    //  ROS_INFO("current encoders: %d %d %d %d %d", current_encoders[0], current_encoders[1], current_encoders[2], current_encoders[3], current_encoders[4]);
    //  ROS_INFO("target encoders:  %d %d %d %d %d", encoders[0], encoders[1], encoders[2], encoders[3], encoders[4]);
    //
    //  kni->moveRobotToEnc(encoders, false);
    //  ros::Duration(2.0).sleep();
    //}

    // ------- wait until start time
    ros::Time start_time = ros::Time(traj->at(0).start_time);
    double time_until_start = (start_time - ros::Time::now()).toSec();

    if (time_until_start < -0.01)
    {
      ROS_WARN("Trajectory started %f s too late! Scheduled: %f, started: %f", -time_until_start, start_time.toSec(), ros::Time::now().toSec());
    }
    else if (time_until_start > 0.0)
    {
      ROS_DEBUG("Sleeping %f seconds until scheduled start of trajectory", time_until_start);
      ros::Time::sleepUntil(start_time);
    }

    // ------- start trajectory
    boost::recursive_mutex::scoped_lock lock(kni_mutex);

    // fix start times
    double delay = ros::Time::now().toSec() - traj->at(0).start_time;
    for (size_t i = 0; i < traj->size(); i++)
    {
      traj->at(i).start_time += delay;
    }

    for (size_t i = 0; i < traj->size(); i++)
    {
      Segment seg = traj->at(i);
      if (seg.splines.size() != joint_names_.size())
      {
        ROS_ERROR("Wrong number of joints in specified trajectory (was: %zu, expected: %zu)!", seg.splines.size(), joint_names_.size());
      }

      // set and start movement
      int activityflag = 0;
      if (i == (traj->size() - 1))
      {
        // last spline, start movement
        activityflag = 1; // no_next
      }
      else if (seg.start_time - traj->at(0).start_time < 0.4)
      {
        // more splines following, do not start movement yet
        activityflag = 2; // no_start
      }
      else
      {
        // more splines following, start movement
        activityflag = 0;
      }

      std::vector<short> polynomial;
      short s_time = round(seg.duration * KNI_TO_ROS_TIME);
      if (s_time <= 0)
        s_time = 1;

      for (size_t j = 0; j < seg.splines.size(); j++)
      {
        // some parts taken from CLMBase::movLM2P
        polynomial.push_back(s_time); // duration

        polynomial.push_back(converter->angle_rad2enc(j, seg.splines[j].target_position)); // target position

        // the four spline coefficients
        // the actual position, round
        polynomial.push_back(round(converter->angle_rad2enc(j, seg.splines[j].coef[0]))); // p0

        // shift to be firmware compatible and round
        polynomial.push_back(round(64 * converter->vel_rad2enc(j, seg.splines[j].coef[1]))); // p1
        polynomial.push_back(round(1024 * converter->acc_rad2enc(j, seg.splines[j].coef[2]))); // p2
        polynomial.push_back(round(32768 * converter->jerk_rad2enc(j, seg.splines[j].coef[3]))); // p3
      }

      // gripper: hold current position
      polynomial.push_back(s_time); // duration
      polynomial.push_back(converter->angle_rad2enc(5, motor_angles_[5])); // target position (angle)
      polynomial.push_back(converter->angle_rad2enc(5, motor_angles_[5])); // p0
      polynomial.push_back(0); // p1
      polynomial.push_back(0); // p2
      polynomial.push_back(0); // p3

      ROS_DEBUG("setAndStartPolyMovement(%d): ", activityflag);

      for (size_t k = 5; k < polynomial.size(); k += 6)
      {
        ROS_DEBUG("   time: %d   target: %d   p0: %d   p1: %d   p2: %d   p3: %d",
            polynomial[k-5], polynomial[k-4], polynomial[k-3], polynomial[k-2], polynomial[k-1], polynomial[k]);
      }

      kni->setAndStartPolyMovement(polynomial, false, activityflag);
    }
    return true;
  }
  catch (WrongCRCException e)
  {
    ROS_ERROR("WrongCRCException: Two threads tried to access the KNI at once. This means that the locking in the Katana node is broken. (exception in executeTrajectory(): %s)", e.message().c_str());
  }
  catch (ReadNotCompleteException e)
  {
    ROS_ERROR("ReadNotCompleteException: Another program accessed the KNI. Please stop it and restart the Katana node. (exception in executeTrajectory(): %s)", e.message().c_str());
  }
  catch (Exception e)
  {
    ROS_ERROR("Unhandled exception in executeTrajectory(): %s", e.message().c_str());
  }
  catch (...)
  {
    ROS_ERROR("Unhandled exception in executeTrajectory()");
  }
  return false;
}

void Katana::freezeRobot()
{
  boost::recursive_mutex::scoped_lock lock(kni_mutex);
  kni->flushMoveBuffers();
  kni->freezeRobot();
}

void Katana::moveGripper(double openingAngle)
{
  if (openingAngle < GRIPPER_CLOSED_ANGLE || GRIPPER_OPEN_ANGLE < openingAngle)
  {
    ROS_ERROR("Desired opening angle %f is out of range [%f, %f]", openingAngle, GRIPPER_CLOSED_ANGLE, GRIPPER_OPEN_ANGLE);
    return;
  }

  try
  {
    boost::recursive_mutex::scoped_lock lock(kni_mutex);
    kni->moveMotorToEnc(GRIPPER_INDEX, converter->angle_rad2enc(GRIPPER_INDEX, openingAngle), false, 100);
  }
  catch (WrongCRCException e)
  {
    ROS_ERROR("WrongCRCException: Two threads tried to access the KNI at once. This means that the locking in the Katana node is broken. (exception in moveGripper(): %s)", e.message().c_str());
  }
  catch (ReadNotCompleteException e)
  {
    ROS_ERROR("ReadNotCompleteException: Another program accessed the KNI. Please stop it and restart the Katana node. (exception in moveGripper(): %s)", e.message().c_str());
  }
  catch (Exception e)
  {
    ROS_ERROR("Unhandled exception in moveGripper(): %s", e.message().c_str());
  }
  catch (...)
  {
    ROS_ERROR("Unhandled exception in moveGripper()");
  }
  return;
}

/* ******************************** helper functions ******************************** */

/**
 * Round to nearest integer.
 */
short inline Katana::round(const double x)
{
  if (x >= 0)
    return (short)(x + 0.5);
  else
    return (short)(x - 0.5);
}

void Katana::calibrate()
{
  // private function only called in constructor, so no locking required
  bool calibrate = false;
  const int encoders = 100;

  kni->unBlock();

  // check if gripper collides in both cases (open and close gripper)
  // note MG: "1" is not the gripper!
  kni->enableCrashLimits();

  try
  {
    kni->moveMotorByEnc(1, encoders);
    ROS_INFO("no calibration required");
  }
  catch (...)
  {
    ROS_INFO("first calibration collision... ");
    try
    {
      kni->moveMotorByEnc(1, -encoders);
      ROS_INFO("no calibration required");
    }
    catch (...)
    {
      ROS_INFO("second calibration collision: calibration required");
      calibrate = true;
    }
  }

  if (calibrate)
  {
    kni->disableCrashLimits();
    kni->calibrate();
    kni->enableCrashLimits();
  }
}

bool Katana::someMotorCrashed()
{
  for (size_t i = 0; i < NUM_MOTORS; i++)
  {
    if (motor_status_[i] == MSF_MOTCRASHED)
      return true;
  }

  return false;
}

bool Katana::allJointsReady()
{
  for (size_t i = 0; i < NUM_JOINTS; i++)
  {
    if ((motor_status_[i] != MSF_DESPOS) && (motor_status_[i] != (MSF_NLINMOV)))
      return false;
  }

  return true;
}

bool Katana::allMotorsReady()
{
  for (size_t i = 0; i < NUM_MOTORS; i++)
  {
    if ((motor_status_[i] != MSF_DESPOS) && (motor_status_[i] != (MSF_NLINMOV)))
      return false;
  }

  return true;
}

void Katana::test_speed()
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

  while (ros::ok())
  {
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
        kni->moveMotorToEnc(i, pos1_encoders);
      }

      do
      {
        idleWait.sleep();
        refreshMotorStatus();
      } while (!allMotorsReady());

      ROS_INFO("Moving to max");
      {
        boost::recursive_mutex::scoped_lock lock(kni_mutex);
        kni->moveMotorToEnc(i, pos2_encoders);
      }

      do
      {
        idleWait.sleep();
        refreshMotorStatus();
      } while (!allMotorsReady());
    }
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
