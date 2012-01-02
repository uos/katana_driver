/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/
#ifndef _HAND_DESCRIPTION_H_
#define _HAND_DESCRIPTION_H_

#include <ros/ros.h>

#include <cmath>

#include <geometry_msgs/Vector3.h>

#include "object_manipulator/tools/exceptions.h"

namespace object_manipulator {

class HandDescription
{
 private:
  //! Node handle in the root namespace
  ros::NodeHandle root_nh_;

  inline std::string getStringParam(std::string name)
  {
    std::string value;
    if (!root_nh_.getParamCached(name, value)) throw MissingParamException(name);
    //ROS_INFO_STREAM("Hand description param " << name << " resolved to " << value);
    return value;
  }

  inline std::vector<std::string> getVectorParam(std::string name)
  {
    XmlRpc::XmlRpcValue list;
    if (!root_nh_.getParamCached(name, list)) throw MissingParamException(name);
    if (list.getType() != XmlRpc::XmlRpcValue::TypeArray) throw BadParamException(name);
    //ROS_INFO_STREAM("Hand description vector param " << name << " resolved to:");
    std::vector<std::string> values;
    for (int32_t i=0; i<list.size(); i++)
    {
      if (list[i].getType() != XmlRpc::XmlRpcValue::TypeString) throw BadParamException(name);
      values.push_back( static_cast<std::string>(list[i]) );
      //ROS_INFO_STREAM("  " << values.back());
    }
    return values;
  }

  inline std::vector<double> getVectorDoubleParam(std::string name)
  {
    XmlRpc::XmlRpcValue list;
    if (!root_nh_.getParamCached(name, list)) throw MissingParamException(name);
    if (list.getType() != XmlRpc::XmlRpcValue::TypeArray) throw BadParamException(name);
    std::vector<double> values;
    for (int32_t i=0; i<list.size(); i++)
    {
      if (list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble) throw BadParamException(name);
      values.push_back( static_cast<double>(list[i]) );
    }
    return values;
  }

 public:
 HandDescription() : root_nh_("~") {}

  inline std::string gripperFrame(std::string arm_name)
  {
    return getStringParam("/hand_description/" + arm_name + "/hand_frame");
  }

  inline std::string robotFrame(std::string arm_name)
  {
    return getStringParam("/hand_description/" + arm_name + "/robot_frame");
  }

  inline std::string attachedName(std::string arm_name)
  {
    return getStringParam("/hand_description/" + arm_name + "/attached_objects_name");
  }

  inline std::string attachLinkName(std::string arm_name)
  {
    return getStringParam("/hand_description/" + arm_name + "/attach_link");
  }

  inline std::string gripperCollisionName(std::string arm_name)
  {
    return getStringParam("/hand_description/" + arm_name + "/hand_group_name");
  }

  inline std::string armGroup(std::string arm_name)
  {
    return getStringParam("/hand_description/" + arm_name + "/arm_group_name");
  }

  inline std::string handDatabaseName(std::string arm_name)
  {
    return getStringParam("/hand_description/" + arm_name + "/hand_database_name");
  }

  inline std::vector<std::string> handJointNames(std::string arm_name)
  {
    return getVectorParam("/hand_description/" + arm_name + "/hand_joints");
  }

  inline std::vector<std::string> gripperTouchLinkNames(std::string arm_name)
  {
    return getVectorParam("/hand_description/" + arm_name + "/hand_touch_links");
  }

  inline std::vector<std::string> fingertipLinks(std::string arm_name)
  {
    return getVectorParam("/hand_description/" + arm_name + "/hand_fingertip_links");
  }

  inline geometry_msgs::Vector3 approachDirection(std::string arm_name)
  {
    std::string name = "/hand_description/" + arm_name + "/hand_approach_direction";
    std::vector<double> values = getVectorDoubleParam(name);
    if ( values.size() != 3 )  throw BadParamException(name);
    double length = sqrt( values[0]*values[0] + values[1]*values[1] + values[2]*values[2] );
    if ( fabs(length) < 1.0e-5 ) throw BadParamException(name);
    geometry_msgs::Vector3 app;
    app.x = values[0] / length;
    app.y = values[1] / length;
    app.z = values[2] / length;
    return app;
  }

};

//! Returns a hand description singleton
inline HandDescription& handDescription()
{
  static HandDescription hand_description;
  return hand_description;
}

} //namespace object_manipulator

#endif
