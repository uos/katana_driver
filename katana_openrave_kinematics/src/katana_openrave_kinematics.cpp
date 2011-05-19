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
 *  Created on: 19.5.2011
 *      Author: Henning Deeken <hdeeken@uos.de>
 */

/**
 * This node provides the get_constraint_aware_ik (kinematics_msgs/GetConstraintAwarePositionIK)
 * service. It just strips away all the constraint aware stuff and calls a the openrave IK service instead.
 *
 * Normally, get_constraint_aware_ik checks the space of potential IK solutions for a
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
#include <motion_planning_msgs/JointPath.h>
#include <motion_planning_msgs/JointPathPoint.h>
#include <orrosplanning/IK.h>


ros::ServiceClient client_;

bool getOpenRaveIK(kinematics_msgs::GetConstraintAwarePositionIK::Request &req,
                   kinematics_msgs::GetConstraintAwarePositionIK::Response &resp) {

  orrosplanning::IK srv;

  // converting the GetConstraintAwarePositionIK::Request to an proper orrosplanning::IK::Request

  srv.request.joint_state = req.ik_request.robot_state.joint_state;
  srv.request.pose_stamped = req.ik_request.pose_stamped;

  client_.call(srv);

  resp.solution.joint_state.header = srv.response.solutions.header;
  resp.solution.joint_state.name = srv.response.solutions.joint_names;

  for(int i = 0; i<srv.response.solutions.points.size;i++){
    resp.solution.joint_state.position[i] = srv.response.solutions.points[i];
  }

  resp.error_code = srv.response.error_code;

  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "katana_openrave_kinematics");
  ros::NodeHandle n;

  ros::NodeHandle pn("~");
  std::string ik_service;

  pn.param<std::string>("ik_service", ik_service, "IK");

  client_ = n.serviceClient<orrosplanning::IK>(ik_service);

  ros::ServiceServer service = pn.advertiseService("get_openrave_ik", getOpenRaveIK);

  ros::spin();

  return 0;
}
