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
 * non_constraint_aware_ik_adapter.cpp
 *
 *  Created on: 09.01.2011
 *      Author: Martin Günther <mguenthe@uos.de>
 */

/**
 * This node provides the get_constraint_aware_ik (kinematics_msgs/GetConstraintAwarePositionIK)
 * service. It just strips away all the constraint aware stuff and calls a normal (non-constraint aware)
 * get_ik (kinematics_msgs/GetPositionIK) service instead.
 *
 * Normally, get_constraint_aware_ik checks the space of potential IK solutions (from get_ik) for a
 * solution that obeys all constraints (i.e., a solution that is not in self-collision or in collision
 * with the environment). That only really makes sense for a redundant (>6 DoF) robot arm. Since the
 * Katana only has 5 DoF, we only get at most one IK solution, so there is nothing to filter.
 *
 * If we return a solution that violates constraints - for instance, that is in self-collision - then
 * move_arm will abort with an error code.  But if the solution is valid, everything should work.
 *
 */

#include <ros/ros.h>

#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetPositionIK.h>


ros::ServiceClient client_;

bool getConstraintAwareIk(kinematics_msgs::GetConstraintAwarePositionIK::Request &req,
                          kinematics_msgs::GetConstraintAwarePositionIK::Response &resp) {

  kinematics_msgs::GetPositionIK srv;
  srv.request.ik_request = req.ik_request;
  srv.request.timeout = req.timeout;

  client_.call(srv);

  resp.error_code = srv.response.error_code;
  resp.solution = srv.response.solution;

  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "non_constraint_aware_ik_adapter");
  ros::NodeHandle n;

  ros::NodeHandle pn("~");
  std::string ik_service;
  pn.param<std::string>("ik_service", ik_service, "get_ik");

  client_ = n.serviceClient<kinematics_msgs::GetPositionIK>(ik_service);

  ros::ServiceServer service = pn.advertiseService("get_constraint_aware_ik", getConstraintAwareIk);

  ros::spin();

  return 0;
}
