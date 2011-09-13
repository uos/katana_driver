/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2011  University of Osnabrück
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
 * katana_teleop_ps3.cpp
 *
 *  Created on: 03.06.2011
 *  Author: Henning Deeken <hdeeken@uos.de>
 *
 **/

#include <katana_teleop/katana_teleop_ps3.h>
#include <math.h>

namespace katana
{

KatanaTeleopPS3::KatanaTeleopPS3() :
  action_client("katana_arm_controller/joint_movement_action", true)
{

  ROS_INFO("KatanaTeleopPS3 starting...");

  ros::NodeHandle n_;
  ros::NodeHandle n_private("~");

  // register service and action clients
  action_client.waitForServer();

  n_private.param<std::string> ("ik_service", ik_service, "get_constraint_aware_ik");
  n_private.param<std::string> ("fk_service", fk_service, "get_fk");
  n_private.param<std::string> ("ik_solver_info", ik_solver_info, "get_ik_solver_info");

  ros::service::waitForService(ik_service);
  ros::service::waitForService(fk_service);
  ros::service::waitForService(ik_solver_info);

  ik_client = n_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK> (ik_service);
  fk_client = n_.serviceClient<kinematics_msgs::GetPositionFK> (fk_service);
  info_client = n_.serviceClient<kinematics_msgs::GetKinematicSolverInfo> (ik_solver_info);

  js_sub_ = n_.subscribe("joint_states", 1000, &KatanaTeleopPS3::jointStateCallback, this);

  ps3joy_sub = n_.subscribe("joy", 100, &KatanaTeleopPS3::ps3joyCallback, this);

  active = true;

}

void KatanaTeleopPS3::jointStateCallback(const sensor_msgs::JointState::ConstPtr& js)
{

  sensor_msgs::JointState incoming_joint_state_;
  incoming_joint_state_.header = js->header;
  incoming_joint_state_.name = js->name;
  incoming_joint_state_.position = js->position;

  kinematics_msgs::GetPositionFK::Request fk_request;
  kinematics_msgs::GetPositionFK::Response fk_response;

  fk_request.header.frame_id = "katana_base_link";
  fk_request.fk_link_names.resize(1);
  fk_request.fk_link_names[0] = "katana_gripper_tool_frame";
  fk_request.robot_state.joint_state = incoming_joint_state_;

  if (fk_client.call(fk_request, fk_response))
  {

    if (fk_response.error_code.val == fk_response.error_code.SUCCESS)
    {

      // update the internal variables
      currentState = incoming_joint_state_;

      //TODO: read pose of katana_gripper_tool_frame
      //currentPose = fk_response.pose_stamped;

      for (unsigned int i = 0; i < fk_response.pose_stamped.size(); i++)
      {

        ROS_DEBUG("       Link: %s", fk_response.fk_link_names[i].c_str());
        ROS_DEBUG("   Position: %f %f %f",
            fk_response.pose_stamped[i].pose.position.x,
            fk_response.pose_stamped[i].pose.position.y,
            fk_response.pose_stamped[i].pose.position.z);
        ROS_DEBUG("Orientation: %f %f %f %f",
            fk_response.pose_stamped[i].pose.orientation.x,
            fk_response.pose_stamped[i].pose.orientation.y,
            fk_response.pose_stamped[i].pose.orientation.z,
            fk_response.pose_stamped[i].pose.orientation.w);
      }

    }
    else
    {
      ROS_ERROR("Forward kinematics failed");
    }
  }
  else
  {
    ROS_ERROR("Forward kinematics service call failed");
  }
}

void KatanaTeleopPS3::ps3joyCallback(const joy::Joy::ConstPtr& joy)
{
  bool execute_action = false;


  if (joy->buttons[16] == 1)
  {
    active = false;
  }

  if (joy->buttons[16] == 0)
  {
    savedState = currentState;

  }

  if (joy->buttons[3] == 0)
  {

    goal.jointGoal = savedState;
    execute_action = true;

  }

  else

  {
     if (joy->buttons[8] == 1)
    {
      addGripperGoalPosition("katana_l_finger_joint", -((joy->axes[8] - 1.0) / -2.0));
      execute_action = true;
    }
    else if (joy->buttons[9] == 1)
    {
      addGripperGoalPosition("katana_l_finger_joint", -((joy->axes[8] - 1.0) / -2.0));
      execute_action = true;
    }

    goalPose = currentPose;
    if (abs(joy->axes[0]) > 0 || abs(joy->axes[1]) > 0 || abs(joy->axes[3]) > 0)
    {
      goalPose.pose.position.y += joy->axes[0];

      goalPose.pose.position.x += joy->axes[1];

      goalPose.pose.position.z += joy->axes[3];
    }

  }

  // define the service messages
  kinematics_msgs::GetConstraintAwarePositionIK::Request gcapik_req;
  kinematics_msgs::GetConstraintAwarePositionIK::Response gcapik_res;
  gcapik_req.timeout = ros::Duration(5.0);

  gcapik_req.ik_request.pose_stamped = goalPose;
  gcapik_req.ik_request.robot_state.joint_state = currentState;
  gcapik_req.ik_request.ik_seed_state.joint_state = currentState;

  if (ik_client.call(gcapik_req, gcapik_res))
  {
    if (gcapik_res.error_code.val == gcapik_res.error_code.SUCCESS)
    {

      for (size_t i = 0; i < gcapik_res.solution.joint_state.name.size(); i++)
      {
        goal.jointGoal.name.push_back(gcapik_res.solution.joint_state.name[i]);
        goal.jointGoal.position.push_back(gcapik_res.solution.joint_state.position[i]);
      }

      execute_action = true;

    }
    else
    {
      ROS_ERROR("IK failed, error code: %d", gcapik_res.error_code.val);
    }
  }
  else
  {
    ROS_ERROR("IK service call failed");
  }

  if (execute_action)
  {

    bool finished_within_time = false;

    action_client.sendGoal(goal);
    finished_within_time = action_client.waitForResult(ros::Duration(1.0));

    if (!finished_within_time)
    {
      action_client.cancelGoal();
      ROS_INFO("Timed out achieving goal!");
    }
    else
    {
      actionlib::SimpleClientGoalState state = action_client.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);

      if (success)
        ROS_INFO("Action finished: %s", state.toString().c_str());
      else
        ROS_INFO("Action failed: %s", state.toString().c_str());

      ros::spinOnce();
    }
  }

}

void KatanaTeleopPS3::addGripperGoalPosition(std::string name, float increment)
{


  float gripper_pos;
  if (getCurrentJointPosition(currentState, name, gripper_pos))
    if ((gripper_pos + increment) >= 0.44 && (gripper_pos + increment) <= 0.30)
    {
      goal.jointGoal.name.push_back(name);
      goal.jointGoal.position.push_back(gripper_pos + increment);
    }
    else
      ROS_WARN("gripper position would exceed limits");
  else
    ROS_WARN("could not access gripper joint");

}

bool KatanaTeleopPS3::getCurrentJointPosition(sensor_msgs::JointState &joint_state, std::string &name, float &position)
{

  for (size_t i = 0; i < joint_state.name.size(); i++)
  {

    if (joint_state.name[i] == name)
    {
      position = joint_state.position[i];
      return true;
    }
  }
  return false;
}

void KatanaTeleopPS3::loop(){

  while (active)
  {
      ros::spin();
  }

}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "katana_teleop_ps3");

  katana::KatanaTeleopPS3 ktps3;

  ktps3.loop();


  return 1;

}


// end namespace
