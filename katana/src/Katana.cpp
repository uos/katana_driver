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

// == not implemented yet ==
//
//   TODO:
//   services? actions?
//     moveRobotToDeg bzw. besser movDegrees
//     !!! openGripper / closeGripper       throws MotorOutOfRangeException, MotorTimeoutException, MotorCrashException   (see pr2_gripper_action, pr2_gripper_controller, pr2_gripper_grasp_controller)
//     freezeMotor / freezeRobot        // stops motor / robot movement
//     switchRobotOn / Off   // default: on; switching off is dangerous because the arm will fall to the ground if unsupported
//     unBlock       // unblock robot after a crash (see crashLimits); has to be called after each MotorCrashException
//
//   inverse kinematics:
//     setTcpOffset
//     IkCalculate (see katana450 package)    // throws NoSolutionException
//
//   perhaps too high level:
//     moveRobotLinearTo
//     moveRobotTo
//     setActivatePositionController
//
//   stuff that we might publish or provide as a service:
//     getForce = efforts
//     get... (all the limits)
//
//   setting limits:
//     setMaximumLinearVelocity
//     setMotorVelocityLimit
//     setRobotVelocityLimit
//     setMotorAccelerationLimit
//     setRobotAccelerationLimit
//     setForceLimit
//     setSpeedCollisionLimit
//     setPositionCollisionLimit

namespace katana
{

Katana::Katana(ros::NodeHandle n) :
  AbstractKatana(n)
{
  motor_status_.resize(NUM_MOTORS);

  /* ********* get parameters ********* */
  std::string ip, config_file_path;
  int port;

  ros::param::param<std::string>("~/katana/ip", ip, "192.168.1.1");
  ros::param::param("~/katana/port", port, 5566);
  ros::param::param("~/katana/config_file_path", config_file_path, ros::package::getPath("kni")
      + "/KNI_4.3.0/configfiles450/katana6M90A_G.cfg");

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

  /* ********* calibrate ********* */
  calibrate();
  ROS_INFO("success: katana calibrated");

  refreshEncoders();
}

Katana::~Katana()
{
  // protocol and device are members of kni, so we should be
  // the last ones using it before deleting them
  assert(kni.use_count() == 1);

  kni.reset(); // delete kni, so protocol and device won't be used any more
  delete protocol;
  delete device;
}

void Katana::refreshEncoders()
{
  try
  {
    boost::recursive_mutex::scoped_lock lock(kni_mutex);
    CMotBase* motors = kni->GetBase()->GetMOT()->arr;

    for (size_t i = 0; i < NUM_MOTORS; i++)
    {
      motors[i].recvPVP(); // throws ParameterReadingException

      const TMotPVP* pvp = motors[i].GetPVP();

      motor_angles_[i] = angle_enc2rad(i, pvp->pos);
      motor_velocities_[i] = vel_acc_jerk_enc2rad(i, pvp->vel); // TODO: test
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

/**
 * Return the position of the tool center point as calculated by the KNI
 *
 * @param refreshEncoders true = read the current encoders from the robot first, false = use cached values
 * @return a vector <x, y, z, r, p, y>; xyz in [m], rpy in [rad]
 */
std::vector<double> Katana::getCoordinates()
{
  const double KNI_TO_ROS_LENGTH = 0.001; // the conversion factor from KNI coordinates (in mm) to ROS coordinates (in m)

  double kni_z, kni_x1, kni_z2;
  std::vector<double> pose(6, 0.0);

  try
  {
    boost::recursive_mutex::scoped_lock lock(kni_mutex);

    kni->getCoordinates(pose[0], pose[1], pose[2], kni_z, kni_x1, kni_z2, false);

    // zyx = yaw, pitch, roll = pose[5], pose[4], pose[3]
    EulerTransformationMatrices::zxz_to_zyx_angles(kni_z, kni_x1, kni_z2, pose[5], pose[4], pose[3]);

    pose[0] = pose[0] * KNI_TO_ROS_LENGTH;
    pose[1] = pose[1] * KNI_TO_ROS_LENGTH;
    pose[2] = pose[2] * KNI_TO_ROS_LENGTH;
  }
  catch (WrongCRCException e)
  {
    ROS_ERROR("WrongCRCException: Two threads tried to access the KNI at once. This means that the locking in the Katana node is broken. (exception in getCoordinates(): %s)", e.message().c_str());
  }
  catch (ReadNotCompleteException e)
  {
    ROS_ERROR("ReadNotCompleteException: Another program accessed the KNI. Please stop it and restart the Katana node. (exception in getCoordinates(): %s)", e.message().c_str());
  }
  catch (Exception e)
  {
    ROS_ERROR("Unhandled exception in getCoordinates(): %s", e.message().c_str());
  }
  catch (...)
  {
    ROS_ERROR("Unhandled exception in getCoordinates()");
  }

  return pose;
}

/**
 * Sends the spline parameters to the Katana.
 *
 * @param traj
 */
bool Katana::executeTrajectory(boost::shared_ptr<SpecifiedTrajectory> traj, ros::Time start_time)
{
  try
  {
    ROS_INFO("Motor status: %d, %d, %d, %d, %d, %d", motor_status_[0], motor_status_[1], motor_status_[2], motor_status_[3], motor_status_[4], motor_status_[5]);
    // Motor status: 8, 8, 8, 8, 8, 8

    // ------- check if motors are blocked
    if (someMotorCrashed())
    {
      ROS_ERROR("Motors are crashed before executing trajectory! Unblocking and aborting trajectory.");
      return false;
    }

    // ------- wait until all motors idle
    ros::Rate idleWait(100);
    while (!allJointsReady())
    {
      idleWait.sleep();
    }

    //// ------- move to start position
    //{
    //  assert(traj->size() > 0);
    //  assert(traj->at(0).splines.size() == NUM_JOINTS);
    //
    //  boost::recursive_mutex::scoped_lock lock(kni_mutex);
    //  std::vector<int> encoders;
    //
    //  for (size_t i = 0; i < NUM_JOINTS; i++) {
    //    encoders.push_back(angle_rad2enc(i, traj->at(0).splines[i].coef[0]));
    //  }
    //
    //  std::vector<int> current_encoders = kni->getRobotEncoders(true);
    //  ROS_INFO("current encoders: %d %d %d %d %d", current_encoders[0], current_encoders[1], current_encoders[2], current_encoders[3], current_encoders[4]);
    //  ROS_INFO("target encoders:  %d %d %d %d %d", encoders[0], encoders[1], encoders[2], encoders[3], encoders[4]);
    //
    //  kni->moveRobotToEnc(encoders, false);
    //  ros::Time move_time = ros::Time::now() + ros::Duration(2.0);
    //  ros::Time::sleepUntil(move_time);
    //}

    // ------- wait until start time
    ros::Time::sleepUntil(start_time);
    if ((ros::Time::now() - start_time).toSec() > 0.01)
      ROS_WARN("Trajectory started %f s too late! Scheduled: %f, started: %f", (ros::Time::now() - start_time).toSec(), start_time.toSec(), ros::Time::now().toSec());

    // ------- start trajectory
    boost::recursive_mutex::scoped_lock lock(kni_mutex);

    // fix start times
    double delay = ros::Time::now().toSec() - traj->at(0).start_time;
    for (size_t i = 0; i < traj->size(); i++) {
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
      double s_time = seg.duration * 100;
      for (size_t j = 0; j < seg.splines.size(); j++)
      {
        // some parts taken from CLMBase::movLM2P
        polynomial.push_back((short)floor(s_time + 0.5)); // duration

        polynomial.push_back(angle_rad2enc(j, seg.splines[j].target_position)); // target position

        // the four spline coefficients
        // the actual position, round
        polynomial.push_back(angle_rad2enc(j, seg.splines[j].coef[0])); // p0

        // shift to be firmware compatible and round
        polynomial.push_back(round(64 * (vel_acc_jerk_rad2enc(j, seg.splines[j].coef[1]) / 100.0))); // p1
        polynomial.push_back(round(1024 * (vel_acc_jerk_rad2enc(j, seg.splines[j].coef[2]) / 10000.0))); // p2
        polynomial.push_back(round(32768 * (vel_acc_jerk_rad2enc(j, seg.splines[j].coef[3]) / 1000000.0))); // p3

        short endpos = angle_rad2enc(j, seg.splines[j].coef[0])
            + vel_acc_jerk_rad2enc(j, seg.splines[j].coef[1]) * seg.duration
            + vel_acc_jerk_rad2enc(j, seg.splines[j].coef[2]) * pow(seg.duration, 2)
            + vel_acc_jerk_rad2enc(j, seg.splines[j].coef[3]) * pow(seg.duration, 3);

        ROS_INFO("target: %d, endpos: %d", angle_rad2enc(j, seg.splines[j].target_position), endpos);

      }

      // gripper
      polynomial.push_back((short)floor(s_time + 0.5)); // duration
      polynomial.push_back(angle_rad2enc(5, motor_angles_[5])); // target position (angle)
      polynomial.push_back(angle_rad2enc(5, motor_angles_[5])); // p0
      polynomial.push_back(0); // p1
      polynomial.push_back(0); // p2
      polynomial.push_back(0); // p3

      ROS_INFO("setAndStartPolyMovement(%d): ", activityflag);

      for (size_t k = 5; k < polynomial.size(); k += 6)
      {
        ROS_INFO("   time: %d   target: %d   p0: %d   p1: %d   p2: %d   p3: %d",
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

/* ******************************** conversions ******************************** */
short Katana::angle_rad2enc(int index, double angle)
{
  // no access to Katana hardware, so no locking required
  const TMotInit* param = kni->GetBase()->GetMOT()->arr[index].GetInitialParameters();

  if (index == NUM_MOTORS - 1) // gripper
    angle = (angle - URDF_GRIPPER_CLOSED_ANGLE) / KNI_TO_URDF_GRIPPER_FACTOR + KNI_GRIPPER_CLOSED_ANGLE;

  return ((param->angleOffset - angle) * (double)param->encodersPerCycle * (double)param->rotationDirection) / (2.0
      * M_PI) + param->encoderOffset;
}

double Katana::angle_enc2rad(int index, int encoders)
{
  // no access to Katana hardware, so no locking required
  const TMotInit* param = kni->GetBase()->GetMOT()->arr[index].GetInitialParameters();

  double result = param->angleOffset - (((double)encoders - (double)param->encoderOffset) * 2.0 * M_PI)
      / ((double)param->encodersPerCycle * (double)param->rotationDirection);

  if (index == NUM_MOTORS - 1) // gripper
  {
    result = (result - KNI_GRIPPER_CLOSED_ANGLE) * KNI_TO_URDF_GRIPPER_FACTOR + URDF_GRIPPER_CLOSED_ANGLE;
  }

  return result;
}

/**
 * Conversions for velocity, acceleration and jerk (first derivative of acceleration).
 * Basically the same as for angle, but without the offsets.
 */
short Katana::vel_acc_jerk_rad2enc(int index, double vel_acc_jerk)
{
  // no access to Katana hardware, so no locking required
  const TMotInit* param = kni->GetBase()->GetMOT()->arr[index].GetInitialParameters();

  if (index == NUM_MOTORS - 1) // gripper
    vel_acc_jerk = vel_acc_jerk / KNI_TO_URDF_GRIPPER_FACTOR;

  return ((-vel_acc_jerk) * (double)param->encodersPerCycle * (double)param->rotationDirection) / (2.0 * M_PI);
}

double Katana::vel_acc_jerk_enc2rad(int index, short encoders)
{
  // no access to Katana hardware, so no locking required
  const TMotInit* param = kni->GetBase()->GetMOT()->arr[index].GetInitialParameters();

  double result = -((double)encoders * 2.0 * M_PI) / ((double)param->encodersPerCycle
      * (double)param->rotationDirection);

  if (index == NUM_MOTORS - 1) // gripper
  {
    result = result * KNI_TO_URDF_GRIPPER_FACTOR;
  }

  return result;
}


/* ******************************** helper functions ******************************** */

short Katana::round(const double x)
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
  bool motorsCrashed = false;

  for (size_t i = 0; i < NUM_MOTORS; i++)
  {
    if (motor_status_[i] == MSF_MOTCRASHED)
    {
      boost::recursive_mutex::scoped_lock lock(kni_mutex);
      motorsCrashed = true;
      kni->unBlock();
      break;
    }
  }

  return motorsCrashed;
}

bool Katana::allJointsReady()
{
  // taken from CLMBase::movLM2P()
  // check if the robot buffer is ready to receive a new linear movement
  bool jointsReady = true;

  for (size_t i = 0; i < NUM_JOINTS; i++)
  {
    jointsReady &= motor_status_[i] != MSF_LINMOV; // TODO: is this really the correct state to check?
  }

  return jointsReady;
}

}
