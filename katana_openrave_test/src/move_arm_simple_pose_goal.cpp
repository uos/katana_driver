#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ompl/util/RandomNumbers.h>
#include <move_arm_msgs/MoveArmAction.h>
#include <move_arm_msgs/utils.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
int counter;
int success_counter;

void move_to_pose(geometry_msgs::Pose & desired_pose,
                  actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> & move_arm)
{

  counter++;

  motion_planning_msgs::SimplePoseConstraint simple_pose_constraint;

  simple_pose_constraint.header.frame_id = "katana_base_link";

  simple_pose_constraint.link_name = "katana_gripper_tool_frame";

  simple_pose_constraint.pose = desired_pose;

  simple_pose_constraint.absolute_position_tolerance.x = 0.03;
  simple_pose_constraint.absolute_position_tolerance.y = 0.03;
  simple_pose_constraint.absolute_position_tolerance.z = 0.03;
  simple_pose_constraint.absolute_roll_tolerance = 3.14;
  simple_pose_constraint.absolute_pitch_tolerance = 3.14;
  simple_pose_constraint.absolute_yaw_tolerance = 3.14;

  /*
   simple_pose_constraint.absolute_roll_tolerance = 3.14;
   simple_pose_constraint.absolute_pitch_tolerance = 3.14;
   simple_pose_constraint.absolute_yaw_tolerance = 3.14;
   */
  simple_pose_constraint.orientation_constraint_type = 0; //HEADER_FRAME
  // simple_pose_constraint.orientation_constraint_type =  1; //LINK_FRAME


  move_arm_msgs::MoveArmGoal goal;

  goal.motion_plan_request.group_name = "arm";
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
  goal.motion_plan_request.allowed_planning_time = ros::Duration(15.0);

  move_arm_msgs::addGoalConstraintToMoveArmGoal(simple_pose_constraint, goal);

  bool finished_within_time = false;

  move_arm.sendGoal(goal);

  finished_within_time = move_arm.waitForResult(ros::Duration(15.5));

  if (!finished_within_time)
  {
    move_arm.cancelGoal();
    ROS_INFO("Timed out achieving goal!");
  }
  else
  {
    actionlib::SimpleClientGoalState state = move_arm.getState();
    bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
    if (success)
    {

      success_counter++;
      btScalar roll, pitch, yaw;
      btQuaternion q = btQuaternion(simple_pose_constraint.pose.orientation.x,
                                    simple_pose_constraint.pose.orientation.y,
                                    simple_pose_constraint.pose.orientation.z,
                                    simple_pose_constraint.pose.orientation.w);

      btMatrix3x3(q).getRPY(roll, pitch, yaw);

      ROS_INFO("Action finished: %s",state.toString().c_str());
      ROS_INFO("xyz: %f / %f / %f ",
          simple_pose_constraint.pose.position.x,
          simple_pose_constraint.pose.position.y,
          simple_pose_constraint.pose.position.z);
      ROS_INFO("quad: %f / %f / %f / %f",
          simple_pose_constraint.pose.orientation.x,
          simple_pose_constraint.pose.orientation.y,
          simple_pose_constraint.pose.orientation.z,
          simple_pose_constraint.pose.orientation.w);
      ROS_INFO("rpy: %f / %f / %f",roll, pitch, yaw);

      ROS_INFO("Trial #%d - succeeded: %f", counter, (double) success_counter/counter);

      tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(simple_pose_constraint.pose.position.x, simple_pose_constraint.pose.position.y,
                                      simple_pose_constraint.pose.position.z));

      transform.setRotation(tf::Quaternion(simple_pose_constraint.pose.orientation.x,
                                           simple_pose_constraint.pose.orientation.y,
                                           simple_pose_constraint.pose.orientation.z,
                                           simple_pose_constraint.pose.orientation.w));

      ros::Rate loop_rate1(100);
      ros::Time end_time = ros::Time::now() + ros::Duration(1.0);

      while (ros::ok() && ros::Time::now() < end_time)
      {
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "katana_base_link", "katana_goal_pose"));
        loop_rate1.sleep();
      }
    }
    else
    {
/*
      tf::TransformBroadcaster br2;
      tf::Transform transform2;

      transform2.setOrigin(tf::Vector3(simple_pose_constraint.pose.position.x, simple_pose_constraint.pose.position.y,
                                      simple_pose_constraint.pose.position.z));

      transform2.setRotation(tf::Quaternion(simple_pose_constraint.pose.orientation.x,
                                           simple_pose_constraint.pose.orientation.y,
                                           simple_pose_constraint.pose.orientation.z,
                                           simple_pose_constraint.pose.orientation.w));

      ros::Rate loop_rate2(100);
      ros::Time end_time = ros::Time::now() + ros::Duration(0.2);

      while (ros::ok() && ros::Time::now() < end_time)
      {

        br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "katana_base_link", "katana_missed_goal_pose"));

        loop_rate2.sleep();
      }
*/
      ROS_DEBUG_THROTTLE(1.0, "Action failed: %s (this is expected to happen a lot)",state.toString().c_str());
    }
  }
}

  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "move_arm_pose_goal_test");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> move_arm("move_arm", true);
    move_arm.waitForServer();
    ROS_INFO("Connected to server");

    counter = 0;

    while (nh.ok())
    {

      ///
      /// RANDOMLY GENERATED POSES
      ///

      geometry_msgs::Pose pose1;

      ompl::RNG rng = ompl::RNG();

      pose1.position.x = rng.uniformReal(-0.48, 0.48);
      pose1.position.y = rng.uniformReal(-0.48, 0.48);
      pose1.position.z = rng.uniformReal(-0.12, 0.68);

      double value2[4];

      rng.quaternion(value2);

      pose1.orientation.x = value2[0];
      pose1.orientation.y = value2[1];
      pose1.orientation.z = value2[2];
      pose1.orientation.w = value2[3];
      // ROS_INFO("Sending move_arm goal for pose 1 (all random)");

      move_to_pose(pose1, move_arm);

      geometry_msgs::Pose pose2;

      pose2.position.x = rng.uniformReal(-0.48, 0.48);
      pose2.position.y = rng.uniformReal(-0.48, 0.48);
      pose2.position.z = rng.uniformReal(-0.12, 0.68);

      double value[4];

      rng.quaternion(value);

      pose2.orientation.x = value2[0];
      pose2.orientation.y = value2[1];
      pose2.orientation.z = value2[2];
      pose2.orientation.w = value2[3];
      // ROS_INFO("Sending move_arm goal for pose 2 (all random)");

      move_to_pose(pose2, move_arm);

      ///
      /// OPEN RAVE APPROVED POSITIONS
      ///

      /*
       //    [ INFO] [1307438122.669621150]: pose position: -0.183997, 0.183440, 0.415803
       //    [ INFO] [1307438122.669667690]: pose orientation: 0.209215, -0.875412, -0.396642, 0.1804

       geometry_msgs::Pose pose1;
       pose1.position.x = -0.183997;
       pose1.position.y = 0.183440;
       pose1.position.z = 0.415803;

       pose1.orientation.x = 0.209215;
       pose1.orientation.y = -0.875412;
       pose1.orientation.z = -0.396642;
       pose1.orientation.w = 0.180439;
       ROS_INFO("Sending move_arm goal for pose 1 (OpenRave approved)");

       move_to_pose(pose1, move_arm);

       // determined with get_constraint_aware_ik_test to have an openrave ik solution
       // [INFO] [1307438247.951109908]: pose position: -0.018995, -0.270915, 0.369523
       // [INFO] [1307438247.951154217]: pose orientation: -0.664450, 0.151834, 0.036378, -0.730842
       // [INFO] [1307438247.951195536]: ...

       geometry_msgs::Pose pose2;
       pose2.position.x = -0.018995;
       pose2.position.y = -0.270915;
       pose2.position.z = 0.369523;

       pose2.orientation.x = -0.664450;
       pose2.orientation.y = 0.151834;
       pose2.orientation.z = 0.036378;
       pose2.orientation.w = -0.730842;

       ROS_INFO("Sending move_arm goal for pose 2 (OpenRave approved)");

       move_to_pose(pose2, move_arm);

       */

      ///
      /// ALL JOINTS 1.5 and 1.8 rad
      ///

      /*
       //    all joints = 1.5 rad, katana_base_link --> katana_gripper_tool_frame (obtained via get_fk)
       //    Position: 0.161907,0.161043,0.517593
       //    Orientation: 0.708431 0.024943 0.704897 0.024953

       geometry_msgs::Pose pose2;
       pose2.position.x = 0.161907;
       pose2.position.y = 0.161043;
       pose2.position.z = 0.517593;

       pose2.orientation.x = 0.708431;
       pose2.orientation.y = 0.024943;
       pose2.orientation.z = 0.704897;
       pose2.orientation.w = 0.024953;

       ROS_INFO("Sending move_arm goal for pose 1 (all joints = 1.5)");

       move_to_pose(pose1, move_arm);

       // all joints = 1.8 rad, katana_base_link --> katana_gripper_tool_frame (obtained via get_fk)
       // Position: 0.139384,0.0476408,0.583725
       // Orientation: 0.720839 -0.078734 0.684101 -0.078725

       geometry_msgs::Pose pose2;
       pose2.position.x = 0.139384;
       pose2.position.y = 0.0476408;
       pose2.position.z = 0.583725;

       pose2.orientation.x = 0.720839;
       pose2.orientation.y = -0.078734;
       pose2.orientation.z = 0.684101;
       pose2.orientation.w = -0.078725;

       ROS_INFO("Sending move_arm goal for pose 2 (all joints = 1.8)");

       move_to_pose(pose2, move_arm);
       */

      ///
      /// LEFT AND RIGHT POSITIONS
      ///

      /*

       // [ INFO] [1308126081.863256828]: Position: -0.193108,-0.421214,0.0501686
       // [ INFO] [1308126081.863297275]: Orientation: -0.544680 0.823614 -0.015894 -0.157259

       geometry_msgs::Pose pose1;

       pose1.position.x = -0.19;
       pose1.position.y = -0.42;
       pose1.position.z = 0.050;

       pose1.orientation.x = -0.54;
       pose1.orientation.y = 0.82;
       pose1.orientation.z = -0.01;
       pose1.orientation.w = -0.15;

       ROS_INFO("Sending move_arm goal for pose 1 (right)");

       move_to_pose(pose1, move_arm);

       // [ INFO] [1308126369.712751011]: Position: 0.162202,-0.419585,0.0597439
       // [ INFO] [1308126369.712814982]: Orientation: 0.837765 -0.535153 -0.035609 0.1024

       geometry_msgs::Pose pose2;

       pose2.position.x = 0.16;
       pose2.position.y = -0.41;
       pose2.position.z = 0.05;

       pose2.orientation.x = 0.83;
       pose2.orientation.y = -0.53;
       pose2.orientation.z = -0.03;
       pose2.orientation.w = 0.10;

       ROS_INFO("Sending move_arm goal for pose 2 (left)");

       move_to_pose(pose2, move_arm);
       */
    }
  }
