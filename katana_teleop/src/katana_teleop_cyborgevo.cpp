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
 * katana_teleop_CyborgEvo.cpp
 *
 *  Created on: 03.06.2011
 *  Author: Henning Deeken <hdeeken@uos.de>
 *
 **/

#include <katana_teleop/katana_teleop_cyborgevo.h>
#include <math.h>

namespace katana
{

KatanaTeleopCyborgEvo::KatanaTeleopCyborgEvo() :
  action_client("katana_arm_controller/joint_movement_action", true)
{

  ROS_INFO("KatanaTeleopCyborgEvo starting...");

  ros::NodeHandle n_;
  ros::NodeHandle n_private("~");

  // register service and action clients
  action_client.waitForServer();

  n_private.param<std::string> ("ik_service", ik_service, "katana_kinematics_constraint_aware/get_openrave_ik");
  //n_private.param<std::string> ("ik_service", ik_service, "non_constraint_aware_ik_adapter/get_constraint_aware_ik");
  n_private.param<std::string> ("fk_service", fk_service, "get_fk");
  n_private.param<std::string> ("ik_solver_info", ik_solver_info, "get_kinematic_solver_info");

  ros::service::waitForService(ik_service);
  ros::service::waitForService(fk_service);
  ros::service::waitForService(ik_solver_info);

  ROS_INFO("KatanaTeleopCyborgEvo connected to all the services...");

  ik_client = n_.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK> (ik_service);
  fk_client = n_.serviceClient<kinematics_msgs::GetPositionFK> (fk_service);
  // info_client = n_.serviceClient<kinematics_msgs::GetKinematicSolverInfo> (ik_solver_info);

  js_sub_ = n_.subscribe("joint_states", 1000, &KatanaTeleopCyborgEvo::jointStateCallback, this);

  cyborgevo_sub = n_.subscribe("joy", 100, &KatanaTeleopCyborgEvo::cyborgevoCallback, this);

  ROS_INFO("KatanaTeleopCyborgEvo subscribed to all the topics..");

  active = true;
  initial = true;

  ROS_INFO("KatanaTeleopCyborgEvo initialized...");
}

int count = 0;

void KatanaTeleopCyborgEvo::jointStateCallback(const sensor_msgs::JointState::ConstPtr& js)
{

  count++;

  ROS_DEBUG("KatanaTeleopCyborgEvo recieved a new JointState...");
  sensor_msgs::JointState incoming_joint_state_;
  incoming_joint_state_.header = js->header;
  incoming_joint_state_.name = js->name;
  incoming_joint_state_.position = js->position;

  if (initial)
    initialState = incoming_joint_state_;


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

    	if(initial){
    	//	initial_RPY_Orientation = fk_response.pose_stamped[0].orientation;
    	}
      // update the internal variables
      currentState = incoming_joint_state_;
      currentPose = fk_response.pose_stamped[0];
      if (count % 100 == 0)
      {
        ROS_ERROR("Actual Pose...");
        for (unsigned int i = 0; i < fk_response.pose_stamped.size(); i++)
        {

          ROS_ERROR("       Link: %s", fk_response.fk_link_names[i].c_str());
          ROS_ERROR("   Position: %f %f %f",
              fk_response.pose_stamped[i].pose.position.x,
              fk_response.pose_stamped[i].pose.position.y,
              fk_response.pose_stamped[i].pose.position.z);
          ROS_ERROR("Orientation: %f %f %f %f",
              fk_response.pose_stamped[i].pose.orientation.x,
              fk_response.pose_stamped[i].pose.orientation.y,
              fk_response.pose_stamped[i].pose.orientation.z,
              fk_response.pose_stamped[i].pose.orientation.w);
        }
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

void KatanaTeleopCyborgEvo::cyborgevoCallback(const joy::Joy::ConstPtr& joy)
{

  ROS_DEBUG("KatanaTeleopCyborgEvo recieved a new Joy command...");

  bool execute_action = false;
  bool request_ik = false;

  if (joy->buttons[10] == 1 && joy->buttons[11] == 1)
  {
    goal.jointGoal = initialState;
    active = false;
    ROS_WARN("Return to Initial State");
    execute_action = true;
  }

  if (joy->buttons[6] == 1 && joy->buttons[8] == 1)
  {
    savedState = currentState;

  }

  if (joy->buttons[7] == 1 && joy->buttons[9] == 1)
  {

    goal.jointGoal = savedState;
    ROS_WARN("Return to Saved State");
    execute_action = true;

  }

  else

  {
    if (joy->axes[5] == 1.0)
    {
      addGripperGoalPosition("katana_l_finger_joint", 5 * 0.017453293);
      ROS_WARN("Close Gripper");
      execute_action = true;
    }
    else if (joy->axes[5] == -1.0)
    {
      addGripperGoalPosition("katana_l_finger_joint", 5 * -0.017453293);
      ROS_WARN("Open Gripper");
      execute_action = true;
    }

    goalPose = currentPose;
    if (abs(joy->axes[0]) > 0.1 ||
    	abs(joy->axes[1]) > 0.1 ||
    	abs(joy->axes[3]) > 0.1
    //	abs(joy->axes[R]) > 0.1 ||
    //	abs(joy->axes[P]) > 0.1 ||
    //	abs(joy->axes[Y]) > 0.1
    	)
    {
      goalPose.pose.position.y += 0.001 * joy->axes[0];
      goalPose.pose.position.x += 0.001 * joy->axes[1];
      goalPose.pose.position.z += 0.001 * joy->axes[3];


/*
 *    add to RPY convert to quad -> orientation = quad
      goalPose.pose.position.y += 0.001 * joy->axes[0];
      goalPose.pose.position.x += 0.001 * joy->axes[1];
      goalPose.pose.position.z += 0.001 * joy->axes[3];
*/

      ROS_WARN("Want to modify Pose");
      request_ik = true;
    }
  }
/*
  ROS_ERROR("Desired Pose...");
  ROS_ERROR("       Link: %s", "katana_gripper_tool_frame");
  ROS_ERROR("   Position: %f %f %f",
      goalPose.pose.position.x,
      goalPose.pose.position.y,
      goalPose.pose.position.z);
  ROS_ERROR("Orientation: %f %f %f %f",
      goalPose.pose.orientation.x,
      goalPose.pose.orientation.y,
      goalPose.pose.orientation.z,
      goalPose.pose.orientation.w);
*/
  if (request_ik)
  {

    // define the service messages
    kinematics_msgs::GetConstraintAwarePositionIK::Request gcapik_req;
    kinematics_msgs::GetConstraintAwarePositionIK::Response gcapik_res;

    gcapik_req.ik_request.ik_link_name = "katana_gripper_tool_frame";
    gcapik_req.ik_request.pose_stamped = goalPose;
    gcapik_req.ik_request.robot_state.joint_state = currentState;
    gcapik_req.ik_request.ik_seed_state.joint_state = currentState;
    gcapik_req.timeout = ros::Duration(5.0);

    ROS_INFO("KatanaTeleopCyborgEvo calls the IK Client...");

    if (ik_client.call(gcapik_req, gcapik_res))
    {
      if (gcapik_res.error_code.val == gcapik_res.error_code.SUCCESS)
      {
        ROS_WARN("IK succeeded...");
        for (size_t i = 0; i < gcapik_res.solution.joint_state.name.size(); i++)
        {
          goal.jointGoal.name.push_back(gcapik_res.solution.joint_state.name[i]);
          goal.jointGoal.position.push_back(gcapik_res.solution.joint_state.position[i]);

          ROS_WARN("Joint: %s - %f", gcapik_res.solution.joint_state.name[i].c_str(), gcapik_res.solution.joint_state.position[i]);
        }
        ROS_WARN("Modify Pose 2");
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
  }

  if (execute_action)
  {
    ROS_WARN("Send Goal...");
    bool finished_within_time = false;

    for (size_t i = 0; i < goal.jointGoal.name.size(); i++)
    {
      ROS_WARN("Joint: %s - %f", goal.jointGoal.name[i].c_str(), goal.jointGoal.position[i]);
    }

    action_client.sendGoal(goal);

    goal.jointGoal.name.clear();
    goal.jointGoal.position.clear();

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

      //ros::spinOnce();
    }
  }

}

void KatanaTeleopCyborgEvo::addGripperGoalPosition(std::string name, float increment)
{

  float gripper_pos;
  if (getCurrentJointPosition(currentState, name, gripper_pos))
  {
    // if ((gripper_pos + increment) >= -0.44 && (gripper_pos + increment) <= 0.30)
    // {
    goal.jointGoal.name.push_back(name);
    goal.jointGoal.position.push_back(gripper_pos + increment);
    // }
    // else
    //   ROS_WARN("gripper position would exceed limits: %f", gripper_pos + increment);
  }
  else
    ROS_WARN("could not access gripper joint");

}

bool KatanaTeleopCyborgEvo::getCurrentJointPosition(sensor_msgs::JointState &joint_state, std::string &name,
                                                    float &position)
{

  for (size_t i = 0; i < joint_state.name.size(); i++)
  {

    if (joint_state.name[i] == name)
    {
      ROS_ERROR("joint: %s - %f", joint_state.name[i].c_str(), joint_state.position[i]);
      position = joint_state.position[i];
      return true;
    }
  }
  return false;
}

void KatanaTeleopCyborgEvo::loop()
{

  ROS_INFO("KatanaTeleopCyborgEvo loops...");
  while (active)
  {
    ros::spin();
  }

}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "katana_teleop_cyborgevo");

  katana::KatanaTeleopCyborgEvo ktc;

  ktc.loop();

  return 1;

}
