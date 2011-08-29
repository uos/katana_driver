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
 * katana_teleop_key.h
 *
 *  Created on: 13.04.2011
 *      Author: Henning Deeken <hdeeken@uos.de>
 */
#ifndef KATANA_TELEOP_KEY_H__
#define KATANA_TELEOP_KEY_H__


#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <katana_msgs/JointMovementAction.h>


#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionServer<katana_msgs::JointMovementAction> JMAS;
typedef actionlib::SimpleActionClient<katana_msgs::JointMovementAction> JMAC;

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77

#define KEYCODE_R 0x72
#define KEYCODE_Q 0x71
#define KEYCODE_I 0x69

#define KEYCODE_PLUS 0x2B
#define KEYCODE_NUMBER 0x23
#define KEYCODE_POINT 0x2E
#define KEYCODE_COMMA 0x2C

#define KEYCODE_0 0x30
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_8 0x38
#define KEYCODE_9 0x39

struct termios cooked, raw;
int kfd = 0;
bool set_initial;


namespace katana{

class KatanaTeleopKey
{

  public:

    KatanaTeleopKey();

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& js);
    bool matchJointGoalRequest(double increment);
    void keyboardLoop();
    void giveInfo();

    ~KatanaTeleopKey()   { }

  private:

    int jointIndex;
    double increment;
    double increment_step;
    double increment_step_scaling;

    std::vector<std::string> joint_names_;
    std::vector<std::string> gripper_joint_names_;
    std::vector<std::string> combined_joints_;

    sensor_msgs::JointState movement_goal_;
    sensor_msgs::JointState current_pose_;
    sensor_msgs::JointState initial_pose_;

    JMAC action_client;
    ros::Subscriber js_sub_;
};
}

#endif /* KATANA_TELEOP_KEY_H__ */
