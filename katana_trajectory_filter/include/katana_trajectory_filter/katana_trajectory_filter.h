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
 * katana_trajectory_filter.h
 *
 *  Created on: 04.02.2011
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#ifndef KATANA_TRAJECTORY_FILTER_H_
#define KATANA_TRAJECTORY_FILTER_H_

#include <ros/ros.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <motion_planning_msgs/JointTrajectoryWithLimits.h>
#include <motion_planning_msgs/JointLimits.h>
#include <motion_planning_msgs/FilterJointTrajectory.h>
#include <motion_planning_msgs/FilterJointTrajectoryWithConstraints.h>

#include <spline_smoother/spline_smoother.h>
#include <pluginlib/class_list_macros.h>

#include <spline_smoother/spline_smoother_utils.h>

#include <vector>
#include <set>

namespace katana_trajectory_filter
{

/**
 * \brief Removes the smallest segments until only MAX_NUM_POINTS remain (currently 16).
 */
template<typename T>
  class KatanaTrajectoryFilter : public spline_smoother::SplineSmoother<T>
  {
  public:
    KatanaTrajectoryFilter();
    virtual ~KatanaTrajectoryFilter();

    virtual bool smooth(const T& trajectory_in, T& trajectory_out) const;

  private:
    void remove_smallest_segments(const T& trajectory_in, T& trajectory_out, const size_t num_points_delete) const;

  };

}

#endif /* KATANA_TRAJECTORY_FILTER_H_ */
