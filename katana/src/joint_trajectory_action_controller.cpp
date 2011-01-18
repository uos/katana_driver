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
 * joint_trajectory_action_controller.cpp
 *
 *  Created on: 07.12.2010
 *      Author: Martin Günther <mguenthe@uos.de>
 *
 * based on joint_trajectory_action_controller.cpp by Stuart Glaser,
 * from the package robot_mechanism_controllers
 */

#include "katana/joint_trajectory_action_controller.h"
#include <fstream>
#include <iostream>
#include <cstdio>

namespace katana
{

JointTrajectoryActionController::JointTrajectoryActionController(ros::NodeHandle &n,
                                                                 boost::shared_ptr<AbstractKatana> katana) :
  katana_(katana), node_(n), action_server_(n, "joint_trajectory_action",
                                            boost::bind(&JointTrajectoryActionController::executeCB, this, _1))
{
  joints_ = katana_->getJointNames();

  // Trajectory and goal constraints
//  node_.param("joint_trajectory_action_node/constraints/goal_time", goal_time_constraint_, 0.0);
  node_.param("joint_trajectory_action_node/constraints/stopped_velocity_tolerance", stopped_velocity_tolerance_, 1e-6);
  goal_constraints_.resize(joints_.size());
//  trajectory_constraints_.resize(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    std::string ns = std::string("joint_trajectory_action_node/constraints") + joints_[i];
    node_.param(ns + "/goal", goal_constraints_[i], 0.02);
//    node_.param(ns + "/trajectory", trajectory_constraints_[i], -1.0);
  }

  // Subscriptions, publishers, services
  sub_command_ = node_.subscribe("command", 1, &JointTrajectoryActionController::commandCB, this);
  serve_query_state_ = node_.advertiseService("query_state", &JointTrajectoryActionController::queryStateService, this);
  controller_state_publisher_ = node_.advertise<pr2_controllers_msgs::JointTrajectoryControllerState> ("state", 1);

  // NOTE: current_trajectory_ is not initialized here, because that will happen in reset_trajectory_and_stop()

  reset_trajectory_and_stop();
}

JointTrajectoryActionController::~JointTrajectoryActionController()
{
  sub_command_.shutdown();
  serve_query_state_.shutdown();
}

/**
 * Sets the current trajectory to a simple "hold position" trajectory
 */
void JointTrajectoryActionController::reset_trajectory_and_stop()
{
  // TODO: send command to KNI (freezeRobot?), wait until stopped

  ros::Time time = ros::Time::now();

  // Creates a "hold current position" trajectory.
  // It's important that this trajectory is always there, because it will be used as a starting point for any new trajectory.
  boost::shared_ptr<SpecifiedTrajectory> hold_ptr(new SpecifiedTrajectory(1));
  SpecifiedTrajectory &hold = *hold_ptr;
  hold[0].start_time = time.toSec() - 0.001;
  hold[0].duration = 0.0;
  hold[0].splines.resize(joints_.size());
  for (size_t j = 0; j < joints_.size(); ++j)
    hold[0].splines[j].coef[0] = katana_->getMotorAngles()[j];

  current_trajectory_ = hold_ptr;
}

void JointTrajectoryActionController::update()
{
  ros::Time time = ros::Time::now();

  std::vector<double> q(joints_.size()), qd(joints_.size()), qdd(joints_.size());

  boost::shared_ptr<const SpecifiedTrajectory> traj_ptr = current_trajectory_;
  if (!traj_ptr)
    ROS_FATAL("The current trajectory can never be null");

  // Only because this is what the code originally looked like.
  const SpecifiedTrajectory &traj = *traj_ptr;

  if (traj.size() == 0)
  {
    ROS_ERROR("No segments in the trajectory");
    return;
  }

  // ------ Finds the current segment

  // Determines which segment of the trajectory to use.
  int seg = -1;
  while (seg + 1 < (int)traj.size() && traj[seg + 1].start_time <= time.toSec())
  {
    ++seg;
  }

  if (seg == -1)
  {
    // ROS_ERROR("No earlier segments.  First segment starts at %.3lf (now = %.3lf)", traj[0].start_time, time.toSec());
    // return;
    seg = 0;
  }

  // ------ Trajectory Sampling

  for (size_t i = 0; i < q.size(); ++i)
  {
    sampleSplineWithTimeBounds(traj[seg].splines[i].coef, traj[seg].duration, time.toSec() - traj[seg].start_time,
                               q[i], qd[i], qdd[i]);
  }

  // ------ Calculate error

  std::vector<double> error(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    error[i] = katana_->getMotorAngles()[i] - q[i];
  }

  // ------ State publishing

  pr2_controllers_msgs::JointTrajectoryControllerState msg;

  // Message containing current state for all controlled joints
  for (size_t j = 0; j < joints_.size(); ++j)
    msg.joint_names.push_back(joints_[j]);
  msg.desired.positions.resize(joints_.size());
  msg.desired.velocities.resize(joints_.size());
  msg.desired.accelerations.resize(joints_.size());
  msg.actual.positions.resize(joints_.size());
  msg.actual.velocities.resize(joints_.size());
  // ignoring accelerations
  msg.error.positions.resize(joints_.size());
  msg.error.velocities.resize(joints_.size());
  // ignoring accelerations

  msg.header.stamp = time;
  for (size_t j = 0; j < joints_.size(); ++j)
  {
    msg.desired.positions[j] = q[j];
    msg.desired.velocities[j] = qd[j];
    msg.desired.accelerations[j] = qdd[j];
    msg.actual.positions[j] = katana_->getMotorAngles()[j];
    msg.actual.velocities[j] = katana_->getMotorVelocities()[j];
    // ignoring accelerations
    msg.error.positions[j] = error[j];
    msg.error.velocities[j] = katana_->getMotorVelocities()[j] - qd[j];
    // ignoring accelerations
  }

  controller_state_publisher_.publish(msg);
}

/**
 * Callback for the "command" topic. There are two ways of sending a trajectory:
 * 1. by publishing a message to the "command" topic (this is what happens here)
 * 2. by using the action server (see executeCB)
 */
void JointTrajectoryActionController::commandCB(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
  // just creates an action from the message and sends it on to the action server

  // create an action client spinning its own thread
  JTAC action_client("joint_trajectory_action", true);
  action_client.waitForServer();

  JTAS::Goal goal;
  goal.trajectory = *(msg.get());

  // fire and forget
  action_client.sendGoal(goal);
}

boost::shared_ptr<SpecifiedTrajectory> JointTrajectoryActionController::calculateTrajectory(
                                                                                            const trajectory_msgs::JointTrajectory &msg,
                                                                                            ros::Time start_time,
                                                                                            std::vector<double> initialJointAngles)
{// TODO: remove initialJointAngles, start_time
  boost::shared_ptr<SpecifiedTrajectory> new_traj_ptr;

  bool allPointsHaveVelocities = true;

  // ------ Checks that the incoming segments have the right number of elements,
  //        determines which spline algorithm to use
  for (size_t i = 0; i < msg.points.size(); i++)
  {
    if (msg.points[i].accelerations.size() != 0 && msg.points[i].accelerations.size() != joints_.size())
    {
      ROS_ERROR("Command point %d has %d elements for the accelerations", (int)i, (int)msg.points[i].accelerations.size());
      return new_traj_ptr; // return null pointer to signal error
    }
    if (msg.points[i].velocities.size() == 0)
    {
      // getCubicSplineCoefficients only works when the desired velocities are already given.
      allPointsHaveVelocities = false;
    }
    else if (msg.points[i].velocities.size() != joints_.size())
    {
      ROS_ERROR("Command point %d has %d elements for the velocities", (int)i, (int)msg.points[i].velocities.size());
      return new_traj_ptr; // return null pointer to signal error
    }
    if (msg.points[i].positions.size() != joints_.size())
    {
      ROS_ERROR("Command point %d has %d elements for the positions", (int)i, (int)msg.points[i].positions.size());
      return new_traj_ptr; // return null pointer to signal error
    }
  }

  // ------ Correlates the joints we're commanding to the joints in the message
  std::vector<int> lookup = makeJointsLookup(msg);
  if (lookup.size() == 0)
    return new_traj_ptr; // return null pointer to signal error


  // ------ convert the boundary conditions to splines
  new_traj_ptr.reset(new SpecifiedTrajectory);
  SpecifiedTrajectory &new_traj = *new_traj_ptr;

  size_t steps = msg.points.size() - 1;

  ROS_DEBUG("steps: %zu", steps);
  assert(steps > 0); // this is checked before

  for (size_t i = 0; i < steps; i++)
  {
    Segment seg;
    seg.splines.resize(joints_.size());
    new_traj.push_back(seg);
  }

  for (size_t j = 0; j < joints_.size(); j++)
  {
    double times[steps + 1], positions[steps + 1], velocities[steps + 1], durations[steps], coeff0[steps],
           coeff1[steps], coeff2[steps], coeff3[steps];

    for (size_t i = 0; i < steps + 1; i++)
    {
      times[i] = start_time.toSec() + msg.points[i].time_from_start.toSec();
      positions[i] = msg.points[i].positions[lookup[j]];
      if (allPointsHaveVelocities)
        velocities[i] = msg.points[i].velocities[lookup[j]];
      ROS_DEBUG("position %zu for joint %zu in message (= our joint %d): %f", i, j, lookup[j], positions[i]);
    }

    for (size_t i = 0; i < steps; i++)
    {
      durations[i] = times[i + 1] - times[i];
    }

    // calculate and store the coefficients
    if (allPointsHaveVelocities)
    {
      ROS_DEBUG("Using getCubicSplineCoefficients()");
      for (size_t i = 0; i < steps; ++i)
      {
        std::vector<double> coeff;
        getCubicSplineCoefficients(positions[i], velocities[i], positions[i + 1], velocities[i + 1], durations[i],
                                   coeff);
        coeff0[i] = coeff[0];
        coeff1[i] = coeff[1];
        coeff2[i] = coeff[2];
        coeff3[i] = coeff[3];
      }
    }
    else
    {
      ROS_DEBUG("Using splineCoefficients()");
      splineCoefficients(steps, times, positions, coeff0, coeff1, coeff2, coeff3);
    }

    for (size_t i = 0; i < steps; ++i)
    {
      new_traj[i].duration = durations[i];
      new_traj[i].start_time = times[i];
      new_traj[i].splines[j].target_position = positions[i + 1];
      new_traj[i].splines[j].coef[0] = coeff0[i];
      new_traj[i].splines[j].coef[1] = coeff1[i];
      new_traj[i].splines[j].coef[2] = coeff2[i];
      new_traj[i].splines[j].coef[3] = coeff3[i];
    }
  }

  ROS_DEBUG("The new trajectory has %d segments", (int)new_traj.size());
  for (size_t i = 0; i < std::min((size_t)20, new_traj.size()); i++)
  {
    ROS_DEBUG("Segment %2zu - start_time: %.3lf   duration: %.3lf", i, new_traj[i].start_time, new_traj[i].duration);
    for (size_t j = 0; j < new_traj[i].splines.size(); ++j)
    {
      ROS_DEBUG("    %.2lf  %.2lf  %.2lf  %.2lf (%s)",
          new_traj[i].splines[j].coef[0],
          new_traj[i].splines[j].coef[1],
          new_traj[i].splines[j].coef[2],
          new_traj[i].splines[j].coef[3],
          joints_[j].c_str());
    }
  }

  // -------- sample trajectory and write to file
  for (size_t j = 0; j < NUM_JOINTS; j++)
  {
    char filename[25];
    sprintf(filename, "/tmp/trajectory-%zu.dat", j);
    std::ofstream trajectory_file(filename, std::ios_base::out);
    trajectory_file.precision(8);
    for (double t = new_traj[0].start_time; t < new_traj.back().start_time + new_traj.back().duration; t += 0.01)
    {
      // Determines which segment of the trajectory to use
      int seg = -1;
      while (seg + 1 < (int)new_traj.size() && new_traj[seg + 1].start_time <= t)
      {
        ++seg;
      }

      assert(seg >= 0);

      double pos_t, vel_t, acc_t;
      sampleSplineWithTimeBounds(new_traj[seg].splines[j].coef, new_traj[seg].duration, t - new_traj[seg].start_time,
                                 pos_t, vel_t, acc_t);

      trajectory_file << t - new_traj[0].start_time << " " << pos_t << " " << vel_t << " " << acc_t << std::endl;
    }

    trajectory_file.close();
  }

  return new_traj_ptr;
}

/**
 * provides the "query_state" service
 */
bool JointTrajectoryActionController::queryStateService(pr2_controllers_msgs::QueryTrajectoryState::Request &req,
                                                        pr2_controllers_msgs::QueryTrajectoryState::Response &resp)
{
  boost::shared_ptr<const SpecifiedTrajectory> traj_ptr;
  traj_ptr = current_trajectory_;
  if (!traj_ptr)
  {
    ROS_FATAL("The current trajectory can never be null");
    return false;
  }
  const SpecifiedTrajectory &traj = *traj_ptr;

  // Determines which segment of the trajectory to use
  int seg = -1;
  while (seg + 1 < (int)traj.size() && traj[seg + 1].start_time <= req.time.toSec())
  {
    ++seg;
  }
  if (seg == -1)
    return false;

  resp.name.resize(joints_.size());
  resp.position.resize(joints_.size());
  resp.velocity.resize(joints_.size());
  resp.acceleration.resize(joints_.size());
  for (size_t j = 0; j < joints_.size(); ++j)
  {
    resp.name[j] = joints_[j];
    sampleSplineWithTimeBounds(traj[seg].splines[j].coef, traj[seg].duration, req.time.toSec() - traj[seg].start_time,
                               resp.position[j], resp.velocity[j], resp.acceleration[j]);
  }

  return true;
}

/**
 * Compares two vectors if they are set-equal (contain same elements in any order)
 */
static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
{
  if (a.size() != b.size())
    return false;

  for (size_t i = 0; i < a.size(); ++i)
  {
    if (count(b.begin(), b.end(), a[i]) != 1)
      return false;
  }
  for (size_t i = 0; i < b.size(); ++i)
  {
    if (count(a.begin(), a.end(), b[i]) != 1)
      return false;
  }

  return true;
}

void JointTrajectoryActionController::executeCB(const JTAS::GoalConstPtr &goal)
{
  if (!setsEqual(joints_, goal->trajectory.joint_names))
  {
    ROS_ERROR("Joints on incoming goal don't match our joints");
    for (size_t i = 0; i < goal->trajectory.joint_names.size(); i++)
    {
      ROS_INFO("  incoming joint %d: %s", (int)i, goal->trajectory.joint_names[i].c_str());
    }
    for (size_t i = 0; i < joints_.size(); i++)
    {
      ROS_INFO("  our joint      %d: %s", (int)i, joints_[i].c_str());
    }
    action_server_.setAborted();
    return;
  }

  // make sure the katana is stopped
  // note: this makes all the "merge old with new trajectory" code in calculateTrajectory obsolete
  reset_trajectory_and_stop();

  // ------ If requested, performs a stop
  if (goal->trajectory.points.empty())
  {
    // reset_trajectory_and_stop();
    action_server_.setSucceeded();
    return;
  }

  // the time to start the trajectory
  ros::Time start_time = ros::Time::now() + ros::Duration(0.1); // we need at least 0.1 seconds for calculation and transmission of trajectory
  if (goal->trajectory.header.stamp != ros::Time(0.0) && goal->trajectory.header.stamp > start_time)
  {
    // trajectory is timed to start later
    start_time = goal->trajectory.header.stamp;
  }

  // calculate new trajectory
  std::vector<double> jointAngles = katana_->getMotorAngles();
  boost::shared_ptr<SpecifiedTrajectory> new_traj = calculateTrajectory(goal->trajectory, start_time, jointAngles);
  if (!new_traj)
  {
    ROS_ERROR("Could not calculate new trajectory, aborting");
    action_server_.setAborted();
    return;
  }
  current_trajectory_ = new_traj;

  // sleep until 0.5 seconds before scheduled start
  ros::Rate rate(10);
  while ((start_time - ros::Time::now()).toSec() > 0.5)
  {
    if (action_server_.isPreemptRequested() || !ros::ok())
    {
      ROS_WARN("Goal canceled by client, aborting");
      action_server_.setPreempted();
      return;
    }
    rate.sleep();
  }

  if (!validTrajectory(*new_traj))
  {
    ROS_ERROR("Computed trajectory did not fulfill all constraints!");
    action_server_.setAborted();
    return;
  }

  bool success = katana_->executeTrajectory(new_traj, start_time); // TODO: doesn't need start_time, is in header stamp
  if (!success)
  {
    ROS_ERROR("Problem while transferring trajectory to Katana arm, aborting");
    action_server_.setAborted();
    return;
  }

  // wait until goal reached
  ros::Rate goalWait(25);
  while (!goalReached())
  {
    goalWait.sleep();
  }

  // that worked out nicely
  action_server_.setSucceeded();
}

/**
 * Checks that we have ended inside the goal constraints.
 */
bool JointTrajectoryActionController::goalReached()
{
  for (size_t i = 0; i < joints_.size(); i++)
  {
    double error = current_trajectory_->back().splines[i].target_position - katana_->getMotorAngles()[i];
    if (goal_constraints_[i] > 0 && fabs(error) > goal_constraints_[i])
      return false;

    // It's important to be stopped if that's desired.
    if (fabs(katana_->getMotorVelocities()[i]) > stopped_velocity_tolerance_)
      return false;
  }
  return true;
}

std::vector<int> JointTrajectoryActionController::makeJointsLookup(const trajectory_msgs::JointTrajectory &msg)
{
  std::vector<int> lookup(joints_.size(), -1); // Maps from an index in joints_ to an index in the msg
  for (size_t j = 0; j < joints_.size(); ++j)
  {
    for (size_t k = 0; k < msg.joint_names.size(); ++k)
    {
      if (msg.joint_names[k] == joints_[j])
      {
        lookup[j] = k;
        break;
      }
    }

    if (lookup[j] == -1)
    {
      ROS_ERROR("Unable to locate joint %s in the commanded trajectory.", joints_[j].c_str());
      return std::vector<int>(); // return empty vector to signal error
    }
  }

  return lookup;
}

/**
 * Checks if the given KNI trajectory fulfills all constraints.
 *
 * @param traj the KNI trajectory to sample
 * @return
 */
bool JointTrajectoryActionController::validTrajectory(const SpecifiedTrajectory &traj)
{
  const double MAX_SPEED = 18000; // encoder per second, TODO: convert to rad/s
  const double MIN_TIME = 0.02; // seconds
  const double EPSILON = 0.000001;

  // ------- check times
  for (size_t seg = 0; seg < traj.size() - 1; seg++)
  {
    if (std::abs(traj[seg].start_time + traj[seg].duration - traj[seg + 1].start_time) > EPSILON)
    {
      ROS_ERROR("start time and duration of segment %zu do not match next segment", seg);
      return false;
    }
  }
  for (size_t seg = 0; seg < traj.size(); seg++)
  {
    if (traj[seg].duration < MIN_TIME)
    {
      ROS_ERROR("duration of segment %zu is too small: %f", seg, traj[seg].duration);
      return false;
    }
  }

  // ------- check conditions at t = 0 and t = N
  for (size_t j = 0; j < traj[0].splines.size(); j++)
  {
    if (std::abs(traj[0].splines[j].coef[1]) > EPSILON)
    {
      ROS_ERROR("Velocity at t = 0 is not 0: %f (joint %zu)", traj[0].splines[j].coef[1], j);
      return false;
    }
  }

  for (size_t j = 0; j < traj[traj.size() - 1].splines.size(); j++)
  {
    size_t seg = traj.size() - 1;
    double vel_t, _;
    sampleSplineWithTimeBounds(traj[seg].splines[j].coef, traj[seg].duration, traj[seg].duration, _, vel_t, _);
    if (std::abs(vel_t) > EPSILON)
    {
      ROS_ERROR("Velocity at t = N is not 0 (joint %zu)", j);
      return false;
    }
  }

  // ------- check for discontinuities between segments
  for (size_t seg = 0; seg < traj.size() - 1; seg++)
  {
    for (size_t j = 0; j < traj[seg].splines.size(); j++)
    {
      double pos_t, vel_t, acc_t;
      sampleSplineWithTimeBounds(traj[seg].splines[j].coef, traj[seg].duration, traj[seg].duration, pos_t, vel_t, acc_t);

      if (std::abs(traj[seg + 1].splines[j].coef[0] - pos_t) > EPSILON)
      {
        ROS_ERROR("Position discontinuity at end of segment %zu (joint %zu)", seg, j);
        return false;
      }
      if (std::abs(traj[seg + 1].splines[j].coef[1] - vel_t) > EPSILON)
      {
        ROS_ERROR("Velocity discontinuity at end of segment %zu (joint %zu)", seg, j);
        return false;
      }
      //  if (std::abs(2.0 * traj[seg + 1].splines[j].coef[2] - acc_t) > EPSILON)
      //  {
      //    ROS_ERROR("Acceleration discontinuity (%f) at end of segment %zu (joint %zu)", std::abs(traj[seg + 1].splines[j].coef[2] - acc_t), seg, j);
      //    return false;
      //  }
    }
  }

  // ------- check position, speed, acceleration limits
  for (double t = traj[0].start_time; t < traj.back().start_time + traj.back().duration; t += 0.01)
  {
    // Determines which segment of the trajectory to use
    int seg = -1;
    while (seg + 1 < (int)traj.size() && traj[seg + 1].start_time <= t)
    {
      ++seg;
    }

    assert(seg >= 0);

    for (size_t j = 0; j < traj[seg].splines.size(); j++)
    {
      double pos_t, vel_t, acc_t;
      sampleSplineWithTimeBounds(traj[seg].splines[j].coef, traj[seg].duration, t - traj[seg].start_time, pos_t, vel_t,
                                 acc_t);

      // TODO later: check position limits (min/max encoders)

      if (std::abs(vel_t) > MAX_SPEED)
      {
        ROS_ERROR("Velocity %f too high at time %f (joint %zu)", vel_t, t, j);
        return false;
      }

      // TODO later: check acceleration limits
    }
  }
  return true;
}

}
