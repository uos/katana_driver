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
 * KatanaNode.cpp
 *
 *  Created on: 11.12.2010
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#include <katana/KatanaNode.h>

namespace katana
{

KatanaNode::KatanaNode(ros::NodeHandle n) :
      nh(n)
{
  bool simulation;
  ros::NodeHandle pn("~");
  pn.param("simulation", simulation, false);

  if (simulation)
    katana.reset(new SimulatedKatana(nh));
  else
    katana.reset(new Katana(nh));
}

KatanaNode::~KatanaNode()
{
}

int KatanaNode::loop()
{
  ros::Rate loop_rate(25);

  JointStatePublisher jointStatePublisher(nh, katana);
  TcpPublisher tcpPublisher(nh, katana);
  JointTrajectoryActionController jointTrajectoryActionController(nh, katana);
  KNIKinematics kni_kinematics(katana);

  while (ros::ok())
  {
    katana->refreshEncoders();
    jointStatePublisher.loopOnce();
    tcpPublisher.loopOnce();
    jointTrajectoryActionController.update();
    kni_kinematics.loopOnce();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "katana");
  ros::NodeHandle n;
  katana::KatanaNode katana_node(n);

  katana_node.loop();

  return 0;
}
