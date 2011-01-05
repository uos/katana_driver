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
 * TestSplines.cpp
 *
 *  Created on: 10.12.2010
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#include <boost/numeric/ublas/lu.hpp>
#include <boost/shared_ptr.hpp>
#include <stdio.h>
#include <vector>
#include <trajectory_msgs/JointTrajectory.h>

using namespace std;

///////////////////////////////////  ROS  ///////////////////////////////////

static inline void generatePowers(int n, double x, double* powers)
{
  powers[0] = 1.0;
  for (int i = 1; i <= n; i++)
  {
    powers[i] = powers[i - 1] * x;
  }
}

static void getQuinticSplineCoefficients(double start_pos, double start_vel, double start_acc, double end_pos,
                                         double end_vel, double end_acc, double time, std::vector<double>& coefficients)
{
  coefficients.resize(6);

  if (time == 0.0)
  {
    coefficients[0] = end_pos;
    coefficients[1] = end_vel;
    coefficients[2] = 0.5 * end_acc;
    coefficients[3] = 0.0;
    coefficients[4] = 0.0;
    coefficients[5] = 0.0;
  }
  else
  {
    double T[6];
    generatePowers(5, time, T);

    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = 0.5 * start_acc;
    coefficients[3] = (-20.0 * start_pos + 20.0 * end_pos - 3.0 * start_acc * T[2] + end_acc * T[2] - 12.0 * start_vel
        * T[1] - 8.0 * end_vel * T[1]) / (2.0 * T[3]);
    coefficients[4] = (30.0 * start_pos - 30.0 * end_pos + 3.0 * start_acc * T[2] - 2.0 * end_acc * T[2] + 16.0
        * start_vel * T[1] + 14.0 * end_vel * T[1]) / (2.0 * T[4]);
    coefficients[5] = (-12.0 * start_pos + 12.0 * end_pos - start_acc * T[2] + end_acc * T[2] - 6.0 * start_vel * T[1]
        - 6.0 * end_vel * T[1]) / (2.0 * T[5]);
  }
}

/**
 * \brief Samples a quintic spline segment at a particular time
 */
static void sampleQuinticSpline(const std::vector<double>& coefficients, double time, double& position,
                                double& velocity, double& acceleration)
{
  // create powers of time:
  double t[6];
  generatePowers(5, time, t);

  position = t[0] * coefficients[0] + t[1] * coefficients[1] + t[2] * coefficients[2] + t[3] * coefficients[3] + t[4]
      * coefficients[4] + t[5] * coefficients[5];

  velocity = t[0] * coefficients[1] + 2.0 * t[1] * coefficients[2] + 3.0 * t[2] * coefficients[3] + 4.0 * t[3]
      * coefficients[4] + 5.0 * t[4] * coefficients[5];

  acceleration = 2.0 * t[0] * coefficients[2] + 6.0 * t[1] * coefficients[3] + 12.0 * t[2] * coefficients[4] + 20.0
      * t[3] * coefficients[5];
}

static void getCubicSplineCoefficients(double start_pos, double start_vel, double end_pos, double end_vel, double time,
                                       std::vector<double>& coefficients)
{
  coefficients.resize(4);

  if (time == 0.0)
  {
    coefficients[0] = end_pos;
    coefficients[1] = end_vel;
    coefficients[2] = 0.0;
    coefficients[3] = 0.0;
  }
  else
  {
    double T[4];
    generatePowers(3, time, T);

    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = (-3.0 * start_pos + 3.0 * end_pos - 2.0 * start_vel * T[1] - end_vel * T[1]) / T[2];
    coefficients[3] = (2.0 * start_pos - 2.0 * end_pos + start_vel * T[1] + end_vel * T[1]) / T[3];
  }
}

static void sampleSplineWithTimeBounds(const std::vector<double>& coefficients, double duration, double time,
                                       double& position, double& velocity, double& acceleration)
{
  if (time < 0)
  {
    double _;
    sampleQuinticSpline(coefficients, 0.0, position, _, _);
    velocity = 0;
    acceleration = 0;
  }
  else if (time > duration)
  {
    double _;
    sampleQuinticSpline(coefficients, duration, position, _, _);
    velocity = 0;
    acceleration = 0;
  }
  else
  {
    sampleQuinticSpline(coefficients, time, position, velocity, acceleration);
  }
}
///////////////////////////////////  KNI  ///////////////////////////////////

void splineCoefficients(int steps, double *timearray, double *encoderarray, double *arr_p1, double *arr_p2,
                        double *arr_p3, double *arr_p4)
{
  int i, j; // countervariables

  // calculate time differences between points and b-coefficients
  double* deltatime = new double[steps];
  double* b = new double[steps];
  for (i = 0; i < steps; i++)
  {
    deltatime[i] = timearray[i + 1] - timearray[i];
    b[i] = 1.0 / deltatime[i];
  }

  // calculate a-coefficients
  double* a = new double[steps - 1];
  for (i = 0; i < (steps - 1); i++)
  {
    a[i] = (2 / deltatime[i]) + (2 / deltatime[i + 1]);
  }

  // build up the right hand side of the linear system
  double* c = new double[steps];
  double* d = new double[steps + 1];
  d[0] = 0; // f_1' and f_n' equal zero
  d[steps] = 0;
  for (i = 0; i < steps; i++)
  {
    c[i] = (encoderarray[i + 1] - encoderarray[i]) / (deltatime[i] * deltatime[i]);
  }
  for (i = 0; i < (steps - 1); i++)
  {
    d[i + 1] = 3 * (c[i] + c[i + 1]);
  }

  // compose A * f' = d
  double** Alin = new double*[steps - 1]; // last column of Alin is right hand side
  for (i = 0; i < (steps - 1); i++)
    Alin[i] = new double[steps];
  // fill with zeros
  for (i = 0; i < (steps - 1); i++)
  {
    for (j = 0; j < steps; j++)
    {
      Alin[i][j] = 0.0;
    }
  }
  // insert values
  for (i = 0; i < (steps - 1); i++)
  {
    if (i == 0)
    {
      Alin[0][0] = a[0];
      Alin[0][1] = b[1];
      Alin[0][steps - 1] = d[1];
    }
    else
    {
      Alin[i][i - 1] = b[i];
      Alin[i][i] = a[i];
      Alin[i][i + 1] = b[i + 1];
      Alin[i][steps - 1] = d[i + 1];
    }
  }

  // solve linear equation
  boost::numeric::ublas::matrix<double> ublas_A(steps - 1, steps - 1);
  boost::numeric::ublas::matrix<double> ublas_b(steps - 1, 1);
  for (i = 0; i < (steps - 1); i++)
  {
    for (j = 0; j < (steps - 1); j++)
    {
      ublas_A(i, j) = Alin[i][j];
    }
    ublas_b(i, 0) = Alin[i][steps - 1];
  }

  boost::numeric::ublas::permutation_matrix<unsigned int> piv(steps - 1);
  lu_factorize(ublas_A, piv);
  lu_substitute(ublas_A, piv, ublas_b);

  // save result in derivatives array
  double* derivatives = new double[steps + 1];
  derivatives[0] = 0;
  for (i = 0; i < (steps - 1); i++)
  {
    derivatives[i + 1] = ublas_b(i, 0);
  }
  derivatives[steps] = 0;
  // build the hermite polynom with difference scheme
  // Q(t) = a0 + (b0 + (c0 + d0 * t) * (t - 1)) * t = a0 + (b0 - c0) * t +
  //   (c0 - d0) * t^2 + d0 * t^3 = p0 + p1 * t + p2 * t^2 + p3 * t^3
  double a0, b0, c0, d0;
  for (i = 0; i < steps; i++)
  {
    a0 = encoderarray[i];
    b0 = encoderarray[i + 1] - a0;
    c0 = b0 - deltatime[i] * derivatives[i];
    d0 = deltatime[i] * (derivatives[i + 1] + derivatives[i]) - 2 * b0;
    arr_p1[i] = a0;
    arr_p2[i] = b0 - c0;
    arr_p3[i] = c0 - d0;
    arr_p4[i] = d0;
  }
}

///
/// converts absolute angles in radian to encoders.
///
template<typename _angleT, typename _encT>
  inline _encT rad2enc(_angleT const& angle, _angleT const& angleOffset, _encT const& epc, _encT const& encOffset,
                       _encT const& rotDir)
  {
    // converting all parameters to _angleT (usually =double)
    _angleT _epc = epc, _rotDir = rotDir, _angleOffset = angleOffset, _encOffset = encOffset;
    return static_cast<_encT> (round(_encOffset + (_angleOffset - angle) * _epc * _rotDir / (2 * M_PI)));
  }

short my_rad2enc(double angle)
{
  double angleOffset = 6.65;
  short encodersPerCycle = 51200, encoderOffset = 31000, rotationDirection = 1;

  return rad2enc(angle, angleOffset, encodersPerCycle, encoderOffset, rotationDirection);
}

///
/// converts encoders to absolute angles in radian
///
template<typename _angleT, typename _encT>
  inline _angleT enc2rad(_encT const& enc, _angleT const& angleOffset, _encT const& epc, _encT const& encOffset,
                         _encT const& rotDir)
  {
    // converting all parameters to _angleT (usually = double)
    _angleT _epc = epc, _rotDir = rotDir, _angleOffset = angleOffset, _encOffset = encOffset, _enc = enc;
    return _angleOffset - (_enc - _encOffset) * 2.0 * M_PI / (_epc * _rotDir);
  }

double my_enc2rad(short enc)
{
  //  encoderOffset         =       "31000";        #  The encoder value the firmware is set to when the mechanical stopper is reached
  //  angleOffset                     =       "6.65";         #  The angle (in degree) which is associated with the mechanical stopper
  //  encodersPerCycle        =       "51200";        #  Number of encoders in one cycle (360°)
  //  angleRange                      =       "339.0";        #  The range between mechanical stoppers (or less if encoder-overflow possible)
  //  rotationDirection       =       "DIR_POSITIVE"; #* This is set DIR_NEGATIVE if angles grow with encoders

  double angleOffset = 6.65;
  short encodersPerCycle = 51200, encoderOffset = 31000, rotationDirection = 1;

  return enc2rad(enc, angleOffset, encodersPerCycle, encoderOffset, rotationDirection);
}

short ros2kni_time(double ros_time)
{
  return ros_time * 100; // kni time is given in 10 ms steps
}

// coef[0] + coef[1]*t + ... + coef[5]*t^5
struct Spline
{
  std::vector<double> coef;

  Spline() :
    coef(6, 0.0)
  {
  }
};

struct Segment
{
  double start_time;
  double duration;
  std::vector<Spline> splines;
};
typedef std::vector<Segment> SpecifiedTrajectory;

boost::shared_ptr<SpecifiedTrajectory> current_trajectory_;

void commandTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
  ros::Time time = ros::Time(1.0);
  size_t num_joints = 1;

  boost::shared_ptr<SpecifiedTrajectory> hold_ptr(new SpecifiedTrajectory(1));
  SpecifiedTrajectory &hold = *hold_ptr;
  hold[0].start_time = time.toSec() - 0.001;
  hold[0].duration = 0.0;
  hold[0].splines.resize(num_joints);
  for (size_t j = 0; j < num_joints; ++j)
//    hold[0].splines[j].coef[0] = my_rad2enc(-0.5);
  hold[0].splines[j].coef[0] = -0.5;

  current_trajectory_ = hold_ptr;

  boost::shared_ptr<SpecifiedTrajectory> new_traj_ptr(new SpecifiedTrajectory);
  SpecifiedTrajectory &new_traj = *new_traj_ptr;

  // ------ Correlates the joints we're commanding to the joints in the message

  std::vector<int> lookup(num_joints, 0);

  // ------ Grabs the trajectory that we're currently following.
  if (!current_trajectory_)
  {
    ROS_FATAL("The current trajectory can never be null");
    return;
  }
  const SpecifiedTrajectory &prev_traj = *current_trajectory_;

  // ------ Copies over the segments from the previous trajectory that are still useful.

  // Useful segments are still relevant after the current time.
  // first_useful: Index of last segment with an end time before now. In other words,
  // the last executed segment.
  int first_useful = -1;
  while (first_useful + 1 < (int)prev_traj.size() && prev_traj[first_useful + 1].start_time <= time.toSec())
  {
    ++first_useful;
  }

  // Useful segments are not going to be completely overwritten by the message's splines.
  // last_useful: Index of last segment with an end time before the message starting time.
  int last_useful = -1;
  double msg_start_time;
  if (msg->header.stamp == ros::Time(0.0))
    msg_start_time = time.toSec();
  else
    msg_start_time = msg->header.stamp.toSec();

  while (last_useful + 1 < (int)prev_traj.size() && prev_traj[last_useful + 1].start_time < msg_start_time)
  {
    ++last_useful;
  }

  // this can happen if msg_start_time < time.toSec(), i.e., the trajectory start point is already in the past
  if (last_useful < first_useful)
    first_useful = last_useful;

  // Copies over the old segments that were determined to be useful.
  for (int i = std::max(first_useful, 0); i <= last_useful; ++i)
  {
    new_traj.push_back(prev_traj[i]);
  }

  // We always save the last segment so that we know where to stop if
  // there are no new segments.
  // this can happen if all segments of the current trajectory start AFTER
  // the new trajectory (in that case first_useful = last_useful = -1), and
  // the start point is in the past
  if (new_traj.size() == 0)
    new_traj.push_back(prev_traj[prev_traj.size() - 1]);

  // ------ Determines when and where the new segments start

  // Finds the end conditions of the final segment
  Segment &last = new_traj[new_traj.size() - 1];
  std::vector<double> prev_positions(num_joints);
  std::vector<double> prev_velocities(num_joints);
  std::vector<double> prev_accelerations(num_joints);

  double t = (msg->header.stamp == ros::Time(0.0) ? time.toSec() : msg->header.stamp.toSec()) - last.start_time;
  ROS_DEBUG("Initial conditions at %.3f for new set of splines:", t);
  for (size_t i = 0; i < num_joints; ++i)
  {
    sampleSplineWithTimeBounds(last.splines[i].coef, last.duration, t, prev_positions[i], prev_velocities[i],
                               prev_accelerations[i]);
    ROS_DEBUG("    %.2lf, %.2lf, %.2lf  (%s)", prev_positions[i], prev_velocities[i],
        prev_accelerations[i], "jointname");
  }

  // ------ Tacks on the new segments

  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;

  std::vector<double> durations(msg->points.size());
  if (msg->points.size() > 0)
    durations[0] = msg->points[0].time_from_start.toSec();
  for (size_t i = 1; i < msg->points.size(); ++i)
    durations[i] = (msg->points[i].time_from_start - msg->points[i - 1].time_from_start).toSec();

  // Checks if we should wrap
  std::vector<double> wrap(num_joints, 0.0);
  if (msg->points[0].positions.empty())
  {
    ROS_ERROR("First point of trajectory has no positions");
    return;
  }
  ROS_DEBUG("wrap:");
  //  for (size_t j = 0; j < num_joints; ++j)
  //  {
  //    if (joints_[j]->getType() == urdf::Joint::CONTINUOUS)
  //    {
  //      double dist = angles::shortest_angular_distance(prev_positions[j], msg->points[0].positions[lookup[j]]);
  //      wrap[j] = (prev_positions[j] + dist) - msg->points[0].positions[lookup[j]];
  //      ROS_DEBUG("    %.2lf  - %s bc dist(%.2lf, %.2lf) = %.2lf", wrap[j], joints_[j]->getName().c_str(),
  //          prev_positions[j], msg->points[0].positions[lookup[j]], dist);
  //    }
  //  }

  for (size_t i = 0; i < msg->points.size(); ++i)
  {
    Segment seg;

    if (msg->header.stamp == ros::Time(0.0))
      seg.start_time = (time + msg->points[i].time_from_start).toSec() - durations[i];
    else
      seg.start_time = (msg->header.stamp + msg->points[i].time_from_start).toSec() - durations[i];
    seg.duration = durations[i];
    //    seg.gh = gh;
    seg.splines.resize(num_joints);

    // Checks that the incoming segment has the right number of elements.

    if (msg->points[i].accelerations.size() != 0 && msg->points[i].accelerations.size() != num_joints)
    {
      ROS_ERROR("Command point %d has %d elements for the accelerations", (int)i, (int)msg->points[i].accelerations.size());
      return;
    }
    if (msg->points[i].velocities.size() != 0 && msg->points[i].velocities.size() != num_joints)
    {
      ROS_ERROR("Command point %d has %d elements for the velocities", (int)i, (int)msg->points[i].velocities.size());
      return;
    }
    if (msg->points[i].positions.size() != num_joints)
    {
      ROS_ERROR("Command point %d has %d elements for the positions", (int)i, (int)msg->points[i].positions.size());
      return;
    }

    // Re-orders the joints in the command to match the internal joint order.

    accelerations.resize(msg->points[i].accelerations.size());
    velocities.resize(msg->points[i].velocities.size());
    positions.resize(msg->points[i].positions.size());
    for (size_t j = 0; j < num_joints; ++j)
    {
      if (!accelerations.empty())
        accelerations[j] = msg->points[i].accelerations[lookup[j]];
      if (!velocities.empty())
        velocities[j] = msg->points[i].velocities[lookup[j]];
      if (!positions.empty())
        positions[j] = msg->points[i].positions[lookup[j]] + wrap[j];
    }

    // Converts the boundary conditions to splines.

    for (size_t j = 0; j < num_joints; ++j)
    {
      if (prev_accelerations.size() > 0 && accelerations.size() > 0)
      {
        getQuinticSplineCoefficients(prev_positions[j], prev_velocities[j], prev_accelerations[j], positions[j],
                                     velocities[j], accelerations[j], durations[i], seg.splines[j].coef);
      }
      else if (prev_velocities.size() > 0 && velocities.size() > 0)
      {
        getCubicSplineCoefficients(prev_positions[j], prev_velocities[j], positions[j], velocities[j], durations[i],
                                   seg.splines[j].coef);
        seg.splines[j].coef.resize(6, 0.0);
      }
      else
      {
        seg.splines[j].coef[0] = prev_positions[j];
        if (durations[i] == 0.0)
          seg.splines[j].coef[1] = 0.0;
        else
          seg.splines[j].coef[1] = (positions[j] - prev_positions[j]) / durations[i];
        seg.splines[j].coef[2] = 0.0;
        seg.splines[j].coef[3] = 0.0;
        seg.splines[j].coef[4] = 0.0;
        seg.splines[j].coef[5] = 0.0;
      }
    }

    // Pushes the splines onto the end of the new trajectory.

    new_traj.push_back(seg);

    // Computes the starting conditions for the next segment

    prev_positions = positions;
    prev_velocities = velocities;
    prev_accelerations = accelerations;
  }

  //  ROS_DEBUG("Last segment goal id: %s", new_traj[new_traj.size()-1].gh->getGoalID().id.c_str());

  // ------ Commits the new trajectory

  if (!new_traj_ptr)
  {
    ROS_ERROR("The new trajectory was null!");
    return;
  }

  current_trajectory_ = new_traj_ptr;

  ROS_DEBUG("The new trajectory has %d segments", (int)new_traj.size());
  for (size_t i = 0; i < std::min((size_t)20, new_traj.size()); ++i)
  {
    ROS_DEBUG("Segment %2d: %.3lf for %.3lf", i, new_traj[i].start_time, new_traj[i].duration);
    for (size_t j = 0; j < new_traj[i].splines.size(); ++j)
    {
      ROS_DEBUG("    %.2lf  %.2lf  %.2lf  %.2lf , %.2lf  %.2lf(%s)",
          new_traj[i].splines[j].coef[0],
          new_traj[i].splines[j].coef[1],
          new_traj[i].splines[j].coef[2],
          new_traj[i].splines[j].coef[3],
          new_traj[i].splines[j].coef[4],
          new_traj[i].splines[j].coef[5],
          "jointname");
    }
  }
}

int main(int argc, char** argv)
{
  // --------- position before trajectory
  double current_pos = -0.5;
  int joint_index = 0;

  // --------- desired trajectory
  //  [trajectory_msgs/JointTrajectory]:
  //  Header header
  //    uint32 seq
  //    time stamp
  //    string frame_id
  //  string[] joint_names
  //  trajectory_msgs/JointTrajectoryPoint[] points
  //    float64[] positions
  //    float64[] velocities
  //    float64[] accelerations
  //    duration time_from_start
  boost::shared_ptr<trajectory_msgs::JointTrajectory> traj(new trajectory_msgs::JointTrajectory());

  traj->header.stamp = ros::Time(3.0); // start time
  traj->joint_names.push_back("testjoint");

  int stp = 8;
  double times[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
  double positions[] = {-0.3, -0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.7};
  double velocities[] = {329.172680, 738.309278, 824.590206, 70.329897, 262.090206,  249.309278, 108.672680, 0.0 };  // from kni, in encoders
  double accelerations[] = {-712.654639, 1121.791237, -1035.510309, 281.250000, -89.489691,  76.708763, -217.345361, 0.0};  // from kni, in encoders

  for (int i = 0; i < stp; i++)
  {
    trajectory_msgs::JointTrajectoryPoint p1;
//    p1.positions.push_back(my_rad2enc(positions[i]));
    p1.positions.push_back(positions[i]);
    p1.velocities.push_back(velocities[i]);
//    p1.accelerations.push_back(accelerations[i]);
    p1.time_from_start = ros::Duration(times[i]);
    traj->points.push_back(p1);
  }
//  0, 14686.000000, 0.000000, 1041.827320, -584.827320
//  1, 15143.000000, 329.172680, -712.654639, 611.481959
//  2, 15371.000000, 738.309278, 1121.791237, -719.100515
//  3, 16512.000000, 824.590206, -1035.510309, 438.920103
//  4, 16740.000000, 70.329897, 281.250000, -123.579897
//  5, 16968.000000, 262.090206, -89.489691, 55.399485
//  6, 17196.000000, 249.309278, 76.708763, -98.018041
//  7, 17424.000000, 108.672680, -217.345361, 108.672680

  // --------- KNI
  int steps = traj->points.size();
  double timearray[steps + 1], encoderarray[steps + 1], arr_p1[steps], arr_p2[steps], arr_p3[steps], arr_p4[steps];

  timearray[0] = ros2kni_time(traj->header.stamp.toSec());
  encoderarray[0] = my_rad2enc(current_pos);

  cout << "-------- time + encoder--------" << endl;
  printf("%d, %f, %f\n", 0, timearray[0], encoderarray[0]);

  for (int i = 0; i < steps; i++)
  {
    timearray[i + 1] = ros2kni_time(traj->header.stamp.toSec() + traj->points[i].time_from_start.toSec());
    encoderarray[i + 1] = my_rad2enc(traj->points[i].positions[joint_index]);

    printf("%d, %f, %f\n", i + 1, timearray[i + 1], encoderarray[i + 1]);
  }

  cout << "-------- spline coeff --------" << endl;
  splineCoefficients(steps, timearray, encoderarray, arr_p1, arr_p2, arr_p3, arr_p4);

  for (int i = 0; i < steps; i++)
  {
    printf("%d, %f, %f, %f, %f\n", i, arr_p1[i], arr_p2[i], arr_p3[i], arr_p4[i]);
  }

  cout << "-------- durations --------" << endl;
  double durations[steps];
  double old_time = 0.0;
  for (int i = 0; i < steps; i++)
  {
    durations[i] = traj->points[i].time_from_start.toSec() - old_time;
    old_time = traj->points[i].time_from_start.toSec();

    printf("%d   %f   %d\n", i, durations[i], ros2kni_time(durations[i]));
  }

  // --------- output
  cout << "-------- kni --------" << endl;
  for (double t = traj->header.stamp.toSec(); t < 14.0; t += 0.01)
  {
    // Determines which segment of the trajectory to use
    if (t < traj->header.stamp.toSec())
    {
      cout << "ERROR: Trying to sample before start of trajectory!";
      return 1;
    }

    size_t seg = 0;
    while (seg < traj->points.size() - 1 && t - traj->header.stamp.toSec() > traj->points[seg].time_from_start.toSec())
    {
      seg++;
    }

    //    static void sampleSplineWithTimeBounds(const std::vector<double>& coefficients, double duration, double time,
    //                                           double& position, double& velocity, double& acceleration)
    double pos_t;
    double vel_t;
    double acc_t;

    std::vector<double> coef;
    coef.push_back(arr_p1[seg]);
    coef.push_back(arr_p2[seg]);
    coef.push_back(arr_p3[seg]);
    coef.push_back(arr_p4[seg]);
    coef.push_back(0.0);
    coef.push_back(0.0);

    //    sampleSplineWithTimeBounds(coef, (double)ros2kni_time(durations[seg]), (double)ros2kni_time(t - traj->header.stamp.toSec() - (traj->points[seg].time_from_start.toSec() - durations[seg])),
    //                               pos_t, vel_t, acc_t);

    sampleSplineWithTimeBounds(coef, durations[seg], t - traj->header.stamp.toSec()
        - (traj->points[seg].time_from_start.toSec() - durations[seg]), pos_t, vel_t, acc_t);

    printf("%f   %d   %f   %f   %f   %f   %f\n", t, seg, my_enc2rad(pos_t), vel_t, acc_t, (double)ros2kni_time(durations[seg]), (double)ros2kni_time(t - traj->header.stamp.toSec() - (traj->points[seg].time_from_start.toSec() - durations[seg])));


  }

  // --------- ROS
  size_t num_joints = 1;
  commandTrajectory(traj);

  cout << "-------- ros coeff --------" << endl;
  for (size_t i = 0; i < current_trajectory_->size(); i++)
  {
    Segment s = current_trajectory_->at(i);
    printf("%d   %f   %f   %f   %f   %f   %f\n", i, s.splines[0].coef[0], s.splines[0].coef[1], s.splines[0].coef[2],
           s.splines[0].coef[3], s.splines[0].coef[4], s.splines[0].coef[5]);
  }

  cout << "-------- ros --------" << endl;
  const SpecifiedTrajectory &traj2 = *current_trajectory_;

  for (double t = traj->header.stamp.toSec(); t < 14.0; t += 0.01)
  {
    // Determines which segment of the trajectory to use
    int seg2 = -1;
    while (seg2 + 1 < (int)traj2.size() && traj2[seg2 + 1].start_time < t)
    {
      ++seg2;
    }
    if (seg2 == -1)
      return false;

    double ros_position, ros_velocity, ros_acceleration;
    for (size_t j = 0; j < num_joints; ++j)
    {
      sampleSplineWithTimeBounds(traj2[seg2].splines[j].coef, traj2[seg2].duration, t - traj2[seg2].start_time,
                                 ros_position, ros_velocity, ros_acceleration);
    }

    printf("%f   %d   %f   %f   %f\n", t, seg2, my_enc2rad(ros_position), ros_velocity, ros_acceleration);
  }

  return 0;
}

