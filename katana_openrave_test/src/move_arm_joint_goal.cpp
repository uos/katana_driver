#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_arm_msgs/MoveArmAction.h>

static const size_t NUM_JOINTS = 5;

bool move_to_joint_goal(std::vector<motion_planning_msgs::JointConstraint> joint_constraints,
                        actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> &move_arm) {

  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");

  goal.motion_plan_request.goal_constraints.joint_constraints = joint_constraints;


  bool finished_within_time = false;
  move_arm.sendGoal(goal);
  finished_within_time = move_arm.waitForResult(ros::Duration(40.0));
  if (!finished_within_time)
  {
    move_arm.cancelGoal();
    ROS_INFO("Timed out achieving goal!");
    return false;
  }
  else
  {
    actionlib::SimpleClientGoalState state = move_arm.getState();
    bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
    if (success)
      ROS_INFO("Action finished: %s",state.toString().c_str());
    else
      ROS_INFO("Action failed: %s",state.toString().c_str());

    return success;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_arm_joint_goal_test");
  ros::NodeHandle nh;
  actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> move_arm("move_arm", true);

  move_arm.waitForServer();
  ROS_INFO("Connected to server");

  std::vector<std::string> names(NUM_JOINTS);
  names[0] = "katana_motor1_pan_joint";
  names[1] = "katana_motor2_lift_joint";
  names[2] = "katana_motor3_lift_joint";
  names[3] = "katana_motor4_lift_joint";
  names[4] = "katana_motor5_wrist_roll_joint";


  std::vector<motion_planning_msgs::JointConstraint> joint_constraints(NUM_JOINTS);

  for (size_t i = 0; i < NUM_JOINTS; ++i)
  {
    joint_constraints[i].joint_name = names[i];
    joint_constraints[i].tolerance_below = 0.1;
    joint_constraints[i].tolerance_above = 0.1;
  }

  while (nh.ok())
  {
    bool success;

    //  == rechts ==
    joint_constraints[0].position = 1.19;
    joint_constraints[1].position = -0.13;  // was +0,13 before
    joint_constraints[2].position = -0.72;
    joint_constraints[3].position = -1.13;
    joint_constraints[4].position = -2.99;

    /* TESTING THE CURRENT OR APPROVED POSES...
   [ INFO] [1307438122.669191590]: Joint: katana_motor1_pan_joint - -0.909271
       [ INFO] [1307438122.669238954]: Joint: katana_motor2_lift_joint - 1.914671
       [ INFO] [1307438122.669285121]: Joint: katana_motor3_lift_joint - -1.654437
       [ INFO] [1307438122.669333631]: Joint: katana_motor4_lift_joint - 0.929276
       [ INFO] [1307438122.669379434]: Joint: katana_motor5_wrist_roll_joint - 0.2730320
    */

     /*   joint_constraints[0].position = -0.909271;
        joint_constraints[1].position = 1.914671;  // was +0,13 before
        joint_constraints[2].position = -1.654437;
        joint_constraints[3].position = 0.929276;
        joint_constraints[4].position = 0.2730320;
*/
    ROS_INFO("Moving to goal 1");
    success = move_to_joint_goal(joint_constraints, move_arm);
    if (!success)
      break;
   /* TESTING THE CURRENT OR APPROVED POSES...
    [ INFO] [1307438247.950864334]: Joint: katana_motor1_pan_joint - -1.845185
           [ INFO] [1307438247.950905301]: Joint: katana_motor2_lift_joint - 1.539664
           [ INFO] [1307438247.950945234]: Joint: katana_motor3_lift_joint - 1.035806
           [ INFO] [1307438247.950985473]: Joint: katana_motor4_lift_joint - -0.495157
           [ INFO] [1307438247.951025535]: Joint: katana_motor5_wrist_roll_joint - -0.475120
        */

    /*        joint_constraints[0].position = -1.845185;
            joint_constraints[1].position = 1.539664;  // was +0,13 before
            joint_constraints[2].position = 1.035806;
            joint_constraints[3].position = -0.495157;
            joint_constraints[4].position = -0.475120;
*/
    //  == links ==
    joint_constraints[0].position = 2.01;
    joint_constraints[1].position = 0.05;  // was +0,14 before
    joint_constraints[2].position = -0.77;
    joint_constraints[3].position = -0.71;
    joint_constraints[4].position = -2.93;

    ROS_INFO("Moving to goal 2");
    success = move_to_joint_goal(joint_constraints, move_arm);
    if (!success)
      break;
  }

  ros::shutdown();
}
