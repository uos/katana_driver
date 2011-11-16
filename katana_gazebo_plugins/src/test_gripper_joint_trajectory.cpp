/*
 * test_gripper_joint_trajectory.cpp
 *
 *  Created on: 24.10.2011
 *      Author: karl
 *
 *      based on: min_max_trajectory.cpp
 */
#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;

/// Constants for gripper fully open or fully closed (should be equal to the value in the urdf description)
static const double GRIPPER_OPEN_ANGLE = 0.30;

/// Constants for gripper fully open or fully closed (should be equal to the value in the urdf description)
static const double GRIPPER_CLOSED_ANGLE = -0.44;

/// A joint angle below this value indicates there is nothing inside the gripper
static const double DEFAULT_GRIPPER_OBJECT_PRESENCE_THRESHOLD = -0.43;

/// The maximum time it takes to open or close the gripper
static const double GRIPPER_OPENING_CLOSING_DURATION = 6.0;

class TestGripperJointTrajectory
{
private:
  // Action client for the joint trajectory action
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  TestGripperJointTrajectory()
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("katana_arm_controller/gripper_joint_trajectory_action", true);

    // wait for action server to come up
    while (!traj_client_->waitForServer(ros::Duration(5.0)) && ros::ok())
    {
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }

    ROS_INFO("The joint_trajectory_action server is available");

  }

  ~TestGripperJointTrajectory()
  {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

  pr2_controllers_msgs::JointTrajectoryGoal closeGoal()
  {
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // set up the joint names
    goal.trajectory.joint_names.push_back("r_finger_joint");
    goal.trajectory.joint_names.push_back("l_finger_joint");

    // set some values

    // num of points
    goal.trajectory.points.resize(2);

    // number of joints in each point
    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].velocities.resize(2);

    goal.trajectory.points[0].positions[0] = GRIPPER_OPEN_ANGLE;
    goal.trajectory.points[0].velocities[0] = 0.0;

    goal.trajectory.points[0].positions[1] = GRIPPER_OPEN_ANGLE;
    goal.trajectory.points[0].velocities[1] = 0.0;

    goal.trajectory.points[0].time_from_start = ros::Duration(0.0);

    // close

    // number of joints in each point
    goal.trajectory.points[1].positions.resize(2);
    goal.trajectory.points[1].velocities.resize(2);

    goal.trajectory.points[1].positions[0] = GRIPPER_CLOSED_ANGLE;
    goal.trajectory.points[1].velocities[0] = 0.0;

    goal.trajectory.points[1].positions[1] = GRIPPER_CLOSED_ANGLE;
    goal.trajectory.points[1].velocities[1] = 0.0;

    goal.trajectory.points[1].time_from_start = ros::Duration(3.0);


    return goal;
  }

  pr2_controllers_msgs::JointTrajectoryGoal openGoal()
  {
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // set up the joint names
    goal.trajectory.joint_names.push_back("r_finger_joint");
    goal.trajectory.joint_names.push_back("l_finger_joint");

    // set some values

    // num of points
    goal.trajectory.points.resize(2);

    // number of joints in each point
    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].velocities.resize(2);

    goal.trajectory.points[0].positions[0] = GRIPPER_CLOSED_ANGLE;
    goal.trajectory.points[0].velocities[0] = 0.0;

    goal.trajectory.points[0].positions[1] = GRIPPER_CLOSED_ANGLE;
    goal.trajectory.points[0].velocities[1] = 0.0;

    goal.trajectory.points[0].time_from_start = ros::Duration(0.0);

    // close

    // number of joints in each point
    goal.trajectory.points[1].positions.resize(2);
    goal.trajectory.points[1].velocities.resize(2);

    goal.trajectory.points[1].positions[0] = GRIPPER_OPEN_ANGLE;
    goal.trajectory.points[1].velocities[0] = 0.0;

    goal.trajectory.points[1].positions[1] = GRIPPER_OPEN_ANGLE;
    goal.trajectory.points[1].velocities[1] = 0.0;

    goal.trajectory.points[1].time_from_start = ros::Duration(3.0);

    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }

};

int main(int argc, char** argv)
{

  // Init the ROS node
  ros::init(argc, argv, "test_gripper_joint_trajectory");

  TestGripperJointTrajectory test;

  ROS_INFO("Start, press CRTL+C to stop!");
  while (ros::ok())
  {

    ROS_INFO("Send closeGoal");
    test.startTrajectory(test.closeGoal());
    while (!test.getState().isDone() && ros::ok())
    {
      usleep(50000);
    }

    ROS_INFO("closeGoal %s", test.getState().toString().c_str());

    usleep(10000000); // 10 sec

    ROS_INFO("Send openGoal");
    test.startTrajectory(test.openGoal());
    while (!test.getState().isDone() && ros::ok())
    {
      usleep(50000);
    }

    ROS_INFO("openGoal %s", test.getState().toString().c_str());

    usleep(10000000); // 10 sec

  }

}

