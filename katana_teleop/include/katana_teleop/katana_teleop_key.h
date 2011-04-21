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
#include <katana/JointMovementAction.h>


#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionServer<katana::JointMovementAction> JMAS;
typedef actionlib::SimpleActionClient<katana::JointMovementAction> JMAC;

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45


struct termios cooked, raw;
int kfd = 0;


namespace katana{

class KatanaTeleopKey
{

  public:

    KatanaTeleopKey();

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& js);

    void keyboardLoop();

    ~KatanaTeleopKey()   { }

  private:

    sensor_msgs::JointState movement_goal_;
    JMAC action_client;

};
}

#endif /* KATANA_TELEOP_KEY_H__ */
