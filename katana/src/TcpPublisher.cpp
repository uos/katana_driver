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
 * TcpPublisher.cpp
 *
 *  Created on: 06.12.2010
 *      Author: Martin Günther <mguenthe@uos.de>
 */

#include "katana/TcpPublisher.h"
#include <geometry_msgs/PoseStamped.h>

namespace katana
{

TcpPublisher::TcpPublisher(ros::NodeHandle n, boost::shared_ptr<AbstractKatana> katana) :
  nh(n), katana(katana)
{
  // do we publish the tool center point frame calculated by the KNI library?
  // this parameter is only useful for debugging, because if everything is
  // set up correctly, katana_gripper_tool_frame (calculated using the URDF
  // description) should be equal to kni_tool_center_point
  ros::param::param("~publish_kni_tcp_frame", publish_kni_tcp_frame, false);
}

TcpPublisher::~TcpPublisher()
{
}

void TcpPublisher::loopOnce()
{
  /* ************** Publish tool center point calculated by the KNI ************** */
  if (publish_kni_tcp_frame)
  {
    tf::Transform transform;

    std::vector<double> pose ;
    // FIXME: std::vector<double> pose = katana->getCoordinates();

    transform.setOrigin(tf::Vector3(pose[0], pose[1], pose[2]));
    transform.setRotation(tf::createQuaternionFromRPY(pose[3], pose[4], pose[5]));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "katana_base_frame", "kni_tool_center_point"));
  }
}

}
