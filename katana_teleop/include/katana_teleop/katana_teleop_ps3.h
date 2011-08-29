/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2011 University of Osnabrück
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
 * katana_teleop_ps3.h
 *
 *  Created on: 03.06.2011
 *      Author: Henning Deeken <hdeeken@uos.de>
 */

#ifndef KATANA_TELEOP_PS3_H__
#define KATANA_TELEOP_PS3_H__

#include <ros/ros.h>
#include <joy/Joy.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>

#include <katana_msgs/JointMovementAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<katana_msgs::JointMovementAction> JMAC;

namespace katana{

class KatanaTeleopPS3
{
   public:

    KatanaTeleopPS3();

    bool matchJointGoalRequest();
    void addGripperGoalPosition(std::string name, float increment);
    bool getCurrentJointPosition(sensor_msgs::JointState &joint_state, std::string &name, float &position);

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& js);
    void ps3joyCallback(const joy::Joy::ConstPtr& joy);

    void loop();

    ~KatanaTeleopPS3(){}

  private:

    bool active;

    katana_msgs::JointMovementGoal goal;
    sensor_msgs::JointState currentState, savedState;
    geometry_msgs::PoseStamped currentPose, goalPose;

    std::string ik_service, fk_service, ik_solver_info;
    ros::ServiceClient info_client, fk_client, ik_client;

    ros::Subscriber js_sub_, ps3joy_sub;

    JMAC action_client;

};

} // end namespace katana
#endif /* KATANA_TELEOP_PS3_H__ */

