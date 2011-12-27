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

KatanaNode::KatanaNode()
{
  bool simulation;
  ros::NodeHandle pn("~");
  pn.param("simulation", simulation, false);

  char *katanaType = getenv("KATANA_TYPE");

  if (simulation)
    katana.reset(new SimulatedKatana());
  else
  {
	if(strcmp(katanaType,"katana_300_6m180") == 0 )
	  katana.reset(new Katana300());
	else
	  katana.reset(new Katana());
  }
}

KatanaNode::~KatanaNode()
{
}

int KatanaNode::loop()
{
  ros::Rate loop_rate(25);

  JointStatePublisher jointStatePublisher(katana);
  JointMovementActionController jointMovementActionController(katana);
  JointTrajectoryActionController jointTrajectoryActionController(katana);
  KatanaGripperGraspController katanaGripperGraspController(katana);

  while (ros::ok())
  {
    katana->refreshEncoders();
    jointStatePublisher.update();
    jointTrajectoryActionController.update();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "katana");
  katana::KatanaNode katana_node;

  katana_node.loop();

  return 0;
}
