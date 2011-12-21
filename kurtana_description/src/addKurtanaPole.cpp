/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *********************************************************************/

/* \author: E. Gil Jones */

#include <ros/ros.h>

#include <mapping_msgs/CollisionObject.h>
#include <geometric_shapes_msgs/Shape.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "addKurtanaPole");

  ros::NodeHandle nh;

  ros::Publisher object_in_map_pub_;
  object_in_map_pub_  = nh.advertise<mapping_msgs::CollisionObject>("collision_object", 10);

  sleep(2);

  //add the cylinder into the collision space
  mapping_msgs::CollisionObject cylinder_object;
  cylinder_object.id = "kurtana_pole";
  cylinder_object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  cylinder_object.header.frame_id = "katana_internal_controlbox_link";
  cylinder_object.header.stamp = ros::Time::now();
  geometric_shapes_msgs::Shape object;
  object.type = geometric_shapes_msgs::Shape::CYLINDER;
  object.dimensions.resize(2);
  object.dimensions[0] = .05;
  object.dimensions[1] = 1.1;
  geometry_msgs::Pose pose;
  pose.position.x = -.19;
  pose.position.y = 0.0;
  pose.position.z = 0.55;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  cylinder_object.shapes.push_back(object);
  cylinder_object.poses.push_back(pose);

  object_in_map_pub_.publish(cylinder_object);

  ROS_INFO("Should have published");

  ros::Duration(2.0).sleep();

  ros::shutdown();

}
