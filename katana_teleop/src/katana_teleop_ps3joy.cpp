#include <ros/ros.h>
#include <joy/Joy.h>
#include <sensor_msgs/JointState.h>

sensor_msgs::JointState currentState;

double max_vel_x, max_rotational_vel;
ros::Publisher vel_pub;
double speed_multiplier;

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& js) {

	kinematics_msgs::GetPositionFK::Request fk_request;
	kinematics_msgs::GetPositionFK::Response fk_response;

	fk_request.header.frame_id = "katana_base_link";
	fk_request.fk_link_names.resize(1);
	fk_request.fk_link_names[0] = "katana_gripper_tool_frame";
	fk_request.robot_state.joint_state = js;

	if (fk_client.call(fk_request, fk_response)) {
		if (fk_response.error_code.val == fk_response.error_code.SUCCESS) {
			for (unsigned int i = 0;
					i < fk_response.pose_stamped.size();
i				++)
				{

					ROS_INFO_STREAM("Link    : " << fk_response.fk_link_names[i].c_str());
					ROS_INFO_STREAM("Position: " <<
					fk_response.pose_stamped[i].pose.position.x << "," <<
					fk_response.pose_stamped[i].pose.position.y << "," <<
					fk_response.pose_stamped[i].pose.position.z);
					ROS_INFO("Orientation: %f %f %f %f",
					fk_response.pose_stamped[i].pose.orientation.x,
					fk_response.pose_stamped[i].pose.orientation.y,
					fk_response.pose_stamped[i].pose.orientation.z,
					fk_response.pose_stamped[i].pose.orientation.w);

					// set the necessary variables
					current_state = js;
					current_pose = fk_response.pose_stamped;

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
}

void ps3joyCallback(const joy::Joy::ConstPtr& joy) {

	katana::JointMovementGoal goal;

	if (joy->buttons[8] == 1) // check for full-speed button
	{
		goal.jointGoal.name.push_back["katana_l_finger_joint"]
		// gripper schließen
	} else if (joy->buttons[9] == 1) // else use half max speed
	{
		//gripper öffnen
	}

goal_pose = current_pose;

if (math.abs(axes[0]) > 0) {
	goal_pose.pose.y + axes[0];
}
if (math.abs(axes[1]) > 0) {
	goal_pose.pose.x + axes[1];
}
if (math.abs(axes[3]) > 0) {
	goal_pose.pose.z + axes[3];
}

// input js into GetPositionIK

// define the service messages
kinematics_msgs::GetPositionIK::Request gpik_req;
kinematics_msgs::GetPositionIK::Response gpik_res;
gpik_req.timeout = ros::Duration(5.0);

gpik_req.ik_request.pose_stamped = goal_pose;
gpik_req.ik_request.robot_state.joint_state = current_state;

if (ik_client.call(gpik_req, gpik_res)) {
	if (gpik_res.error_code.val == gpik_res.error_code.SUCCESS)

		katana::JointMovementGoal goal;
	goal.jointGoal = gpik_res.solution.joint_state;

	bool finished_within_time = false;
	action_client.sendGoal(goal);
	finished_within_time = action_client.waitForResult(ros::Duration(1.0));
	if (!finished_within_time) {
		action_client.cancelGoal();
		ROS_INFO("Timed out achieving goal!");
	} else {
		actionlib::SimpleClientGoalState state = action_client.getState();
		bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
		if (success)
			ROS_INFO("Action finished: %s", state.toString().c_str());
		else
			ROS_INFO("Action failed: %s", state.toString().c_str());

		ROS_INFO("...goal was successfully send!");
		ros::spinOnce();
	}

else ROS_ERROR("Inverse kinematics failed, error code: %d", gpik_res.error_code.val);
}
else
ROS_ERROR("Inverse kinematics service call failed");
}
}

int main(int argc, char** argv) {
ros::init(argc, argv, "kurt_teleop_ps3joy");

ros::NodeHandle nh;
ros::NodeHandle nh_ns("~");

action_client("joint_movement_action", true)
action_client.waitForService();

std::string ik_service;
nh_ns.param<std::string> ("ik_service", ik_service, "get_ik");
client_ = n.serviceClient<kinematics_msgs::GetPositionIK> (ik_service);

ros::service::waitForService("get_kinematic_solver_info");
ros::service::waitForService("get_fk");
ros::service::waitForService("ik_service");
ros::ServiceClient query_client = nh.serviceClient<
		kinematics_msgs::GetKinematicSolverInfo> ("get_kinematic_solver_info");
ros::ServiceClient fk_client =
		nh.serviceClient<kinematics_msgs::GetPositionFK> ("get_fk");

ros::Subscriber js_sub_ = nh.subscribe("joint_states", 1000,
		&KatanaTeleopPS3::jointStateCallback, this);
ros::Subscriber ps3joy_sub = nh.subscribe("joy", 10, ps3joyCallback);

// Gets all of the joints
  XmlRpc::XmlRpcValue joint_names;

  // Gets all of the joints
  if (!n_.getParam("katana_joints", joint_names))
  {
    ROS_ERROR("No joints given. (namespace: %s)", n_.getNamespace().c_str());
  }
  joint_names_.resize(joint_names.size());

  if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Malformed joint specification.  (namespace: %s)", n_.getNamespace().c_str());
  }

  for (size_t i = 0; i < joint_names.size(); ++i)
  {

    XmlRpc::XmlRpcValue &name_value = joint_names[i];

    if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("Array of joint names should contain all strings.  (namespace: %s)",
          n_.getNamespace().c_str());
    }

    joint_names_[i] = (std::string)name_value;

  }

  // Gets all of the gripper joints
  XmlRpc::XmlRpcValue gripper_joint_names;

  // Gets all of the joints
  if (!n_.getParam("katana_gripper_joints", gripper_joint_names))
  {
    ROS_ERROR("No gripper joints given. (namespace: %s)", n_.getNamespace().c_str());
  }

  gripper_joint_names_.resize(gripper_joint_names.size());

  if (gripper_joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Malformed gripper joint specification.  (namespace: %s)", n_.getNamespace().c_str());
  }
  for (size_t i = 0; i < gripper_joint_names.size(); ++i)
  {
    XmlRpc::XmlRpcValue &name_value = gripper_joint_names[i];
    if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("Array of gripper joint names should contain all strings.  (namespace: %s)",
          n_.getNamespace().c_str());
    }

    gripper_joint_names_[i] = (std::string)name_value;
  }

  combined_joints_.resize(joint_names_.size() + gripper_joint_names_.size());

  for (unsigned int i = 0; i < joint_names_.size(); i++)
  {
    combined_joints_[i] = joint_names_[i];
  }

  for (unsigned int i = 0; i < gripper_joint_names_.size(); i++)
  {
    combined_joints_[joint_names_.size() + i] = gripper_joint_names_[i];
  }

ros::spin();
}



bool KatanaTeleopPS3::matchJointGoalRequest()
{

  bool found_match = false;

  for (unsigned int i = 0; i < current_pose_.name.size(); i++)
  {

    if (current_pose_.name[i] == combined_joints_[jointIndex])
    {
      movement_goal_.position.push_back();
      found_match = true;
      break;

    }
  }

  return found_match;
}

