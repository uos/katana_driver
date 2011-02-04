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
 * katana_trajectory_filter.cpp
 *
 *  Created on: 04.02.2011
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#include <katana_trajectory_filter/katana_trajectory_filter.h>

namespace katana_trajectory_filter
{

/// the maximum number of points that the output trajectory should have
static const size_t MAX_NUM_POINTS = 16;

template<typename T>
  KatanaTrajectoryFilter<T>::KatanaTrajectoryFilter()
  {
  }

template<typename T>
  KatanaTrajectoryFilter<T>::~KatanaTrajectoryFilter()
  {
  }

template<typename T>
  bool KatanaTrajectoryFilter<T>::smooth(const T& trajectory_in, T& trajectory_out) const
  {
    bool success = true;
    size_t num_points = trajectory_in.trajectory.points.size();
    trajectory_out = trajectory_in;

    if (!spline_smoother::checkTrajectoryConsistency(trajectory_out))
      return false;

    if (num_points <= MAX_NUM_POINTS)
      // nothing to do
      return true;

    std::vector<std::pair<size_t, double> > segment_durations(num_points - 2);

    // calculate segment_durations, leave out the first segment to keep it intact
    for (size_t i = 1; i < num_points - 1; ++i)
    {
      double duration = (trajectory_in.trajectory.points[i + 1].time_from_start
          - trajectory_in.trajectory.points[i].time_from_start).toSec();

      segment_durations[i - 1] = std::pair<size_t, double>(i, duration);
    }

    for (size_t i = 0; i < segment_durations.size(); i++)
      ROS_DEBUG("segment_durations[%3zu] = <%3zu, %f>", i, segment_durations[i].first, segment_durations[i].second);

    // sort segment_durations by their duration, in ascending order
    std::sort(segment_durations.begin(), segment_durations.end(), boost::bind(&std::pair<size_t, double>::second, _1)
        < boost::bind(&std::pair<size_t, double>::second, _2));

    for (size_t i = 0; i < segment_durations.size(); i++)
      ROS_DEBUG("segment_durations[%3zu] = <%3zu, %f>", i, segment_durations[i].first, segment_durations[i].second);

    // delete the smallest segments until only MAX_NUM_POINTS remain
    std::set<size_t> delete_set;
    for (size_t i = 0; i < num_points - MAX_NUM_POINTS; i++)
    {
      delete_set.insert(segment_durations[i].first);
    }

    for (std::set<size_t>::iterator it = delete_set.begin(); it != delete_set.end(); it++)
      ROS_DEBUG("delete set entry: %zu", *it);

    trajectory_out.trajectory.points.resize(0);
    for (size_t i = 0; i < num_points; i++)
    {
      if (delete_set.find(i) == delete_set.end())
      {
        // segment i is not in the delete set --> copy
        trajectory_out.trajectory.points.push_back(trajectory_in.trajectory.points[i]);
      }
    }

    // delete all velocities and accelerations so they will be re-computed by Katana
    for (size_t i = 0; i < trajectory_out.trajectory.points.size(); ++i)
    {
      trajectory_out.trajectory.points[i].velocities.resize(0);
      trajectory_out.trajectory.points[i].accelerations.resize(0);
    }

    return success;
  }

} // namespace katana_trajectory_filter

//  PLUGINLIB_REGISTER_CLASS(class_name, class_type, filters::FilterBase<T>)
PLUGINLIB_REGISTER_CLASS(KatanaTrajectoryFilterFilterJointTrajectory, katana_trajectory_filter::KatanaTrajectoryFilter<motion_planning_msgs::FilterJointTrajectory::Request>, filters::FilterBase<motion_planning_msgs::FilterJointTrajectory::Request>)
PLUGINLIB_REGISTER_CLASS(KatanaTrajectoryFilterJointTrajectoryWithLimits, katana_trajectory_filter::KatanaTrajectoryFilter<motion_planning_msgs::JointTrajectoryWithLimits>, filters::FilterBase<motion_planning_msgs::JointTrajectoryWithLimits>)
PLUGINLIB_REGISTER_CLASS(KatanaTrajectoryFilterFilterJointTrajectoryWithConstraints, katana_trajectory_filter::KatanaTrajectoryFilter<motion_planning_msgs::FilterJointTrajectoryWithConstraints::Request>, filters::FilterBase<motion_planning_msgs::FilterJointTrajectoryWithConstraints::Request>)
