#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;

class RobotArm
{
private:
  // Action client for the joint trajectory action
  // used to trigger the arm movement action
  TrajClient* traj_client_;

  template<typename _angleT, typename _encT>
    inline _angleT enc2rad(_encT const& enc, _angleT const& angleOffset, _encT const& epc, _encT const& encOffset,
                           _encT const& rotDir)
    {
      // converting all parameters to _angleT (usually = double)
      _angleT _epc = epc, _rotDir = rotDir, _angleOffset = angleOffset, _encOffset = encOffset, _enc = enc;
      return _angleOffset - (_enc - _encOffset) * 2.0 * M_PI / (_epc * _rotDir);
    }

  double my_enc2rad(int encoders)
  {
    //  encoderOffset         =       "31000";        #  The encoder value the firmware is set to when the mechanical stopper is reached
    //  angleOffset                     =       "6.65";         #  The angle (in degree) which is associated with the mechanical stopper
    //  encodersPerCycle        =       "51200";        #  Number of encoders in one cycle (360°)
    //  angleRange                      =       "339.0";        #  The range between mechanical stoppers (or less if encoder-overflow possible)
    //  rotationDirection       =       "DIR_POSITIVE"; #* This is set DIR_NEGATIVE if angles grow with encoders

    double angleOffset = 0.116064; // 6.65° = 0.116 rad
    double encodersPerCycle = 51200, encoderOffset = 31000, rotationDirection = 1;

    return angleOffset - ((double)(encoders - encoderOffset) * 2.0 * M_PI) / (double)(encodersPerCycle
        * rotationDirection);
  }

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm()
  {
    // tell the action client that we want to spin a thread by default
    //    traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
    traj_client_ = new TrajClient("joint_trajectory_action", true);

    // wait for action server to come up
    while (!traj_client_->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~RobotArm()
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

  pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory()
  {
    const size_t NUM_SEGMENTS = 6;
    const size_t NUM_JOINTS = 5;

    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("katana_motor1_pan_joint");
    goal.trajectory.joint_names.push_back("katana_motor2_lift_joint");
    goal.trajectory.joint_names.push_back("katana_motor3_lift_joint");
    goal.trajectory.joint_names.push_back("katana_motor4_lift_joint");
    goal.trajectory.joint_names.push_back("katana_motor5_wrist_roll_joint");

    goal.trajectory.points.resize(NUM_SEGMENTS);

    // First trajectory point
    // Positions (after calibration)
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(NUM_JOINTS);
    goal.trajectory.points[ind].positions[1] = 2.163279155148794 - 0.1;
    goal.trajectory.points[ind].positions[2] = 0.93249037289099745 + 0.1;
    goal.trajectory.points[ind].positions[3] = 1.1156471827982504 + 0.1;
    goal.trajectory.points[ind].positions[4] = 0.20983493639797451 + 0.1;

    // Velocities
    //    goal.trajectory.points[ind].velocities.resize(5);
    //    for (size_t j = 0; j < 5; ++j)
    //    {
    //      goal.trajectory.points[ind].velocities[j] = 0.0;
    //    }

    // Positions
    for (size_t ind = 1; ind < NUM_SEGMENTS; ind++)
    {
      goal.trajectory.points[ind].positions.resize(NUM_JOINTS);
      goal.trajectory.points[ind].positions[1] = goal.trajectory.points[ind - 1].positions[1];
      goal.trajectory.points[ind].positions[2] = goal.trajectory.points[ind - 1].positions[2];
      goal.trajectory.points[ind].positions[3] = goal.trajectory.points[ind - 1].positions[3];
      goal.trajectory.points[ind].positions[4] = goal.trajectory.points[ind - 1].positions[4];
    }

    //    p1_enc: 30069
    //   pp axis 1:       target: 30069   time: 300       pp0: 30969, pp1: 0, pp2: -1024, pp3: 0
    //   pp axis 1:       target: 6789    time: 3880      pp0: 30069, pp1: -3840, pp2: 0, pp3: 0
    //   pp axis 1:       target: 5914    time: 250       pp0: 6789, pp1: -3840, pp2: 1024, pp3: 0
    //   pp axis 1:       target: 5904    time: 10        pp0: 5914, pp1: -640, pp2: 0, pp3: 0
    //   pp axis 1:       target: 5879    time: 50        pp0: 5904, pp1: -640, pp2: 1024, pp3: 0
    //   pp axis 1:       target: 5878    time: 30        pp0: 5879, pp1: 0, pp2: 0, pp3: 0

    goal.trajectory.points[0].time_from_start = ros::Duration(1.0);
    goal.trajectory.points[1].time_from_start = ros::Duration(2.0);
    goal.trajectory.points[2].time_from_start = ros::Duration(3.0);
    goal.trajectory.points[3].time_from_start = ros::Duration(4.0);
    goal.trajectory.points[4].time_from_start = ros::Duration(5.0);
    goal.trajectory.points[5].time_from_start = ros::Duration(6.0);

    //  goal.trajectory.points[0].positions[0] = my_enc2rad(30069);
    //  goal.trajectory.points[1].positions[0] = my_enc2rad(-6789);
    //  goal.trajectory.points[2].positions[0] = my_enc2rad(5914);
    //  goal.trajectory.points[3].positions[0] = my_enc2rad(-5904);
    //  goal.trajectory.points[4].positions[0] = my_enc2rad(5879);
    //  goal.trajectory.points[5].positions[0] = my_enc2rad(-5878);

    goal.trajectory.points[0].positions[0] = my_enc2rad(20499);
    goal.trajectory.points[1].positions[0] = my_enc2rad(20100);
    goal.trajectory.points[2].positions[0] = my_enc2rad(19800);
    goal.trajectory.points[3].positions[0] = my_enc2rad(19300);
    goal.trajectory.points[4].positions[0] = my_enc2rad(19000);
    goal.trajectory.points[5].positions[0] = my_enc2rad(19000);

    ROS_INFO("pos[%d]: %f", 0, goal.trajectory.points[0].positions[0]);
    ROS_INFO("pos[%d]: %f", 1, goal.trajectory.points[1].positions[0]);
    ROS_INFO("pos[%d]: %f", 2, goal.trajectory.points[2].positions[0]);
    ROS_INFO("pos[%d]: %f", 3, goal.trajectory.points[3].positions[0]);
    ROS_INFO("pos[%d]: %f", 4, goal.trajectory.points[4].positions[0]);
    ROS_INFO("pos[%d]: %f", 5, goal.trajectory.points[5].positions[0]);

    //    goal.trajectory.points[ind].positions[0] = goal.trajectory.points[ind - 1].positions[0] + 0.1;
    //    goal.trajectory.points[ind].positions[1] = goal.trajectory.points[ind - 1].positions[1] - 0.1;
    //    goal.trajectory.points[ind].positions[2] = goal.trajectory.points[ind - 1].positions[2] + 0.1;
    //    goal.trajectory.points[ind].positions[3] = goal.trajectory.points[ind - 1].positions[3] + 0.1;
    //    goal.trajectory.points[ind].positions[4] = goal.trajectory.points[ind - 1].positions[4] + 0.1;

    // Velocities
    //    goal.trajectory.points[ind].velocities.resize(NUM_JOINTS);
    //    for (size_t j = 0; j < NUM_JOINTS; ++j)
    //    {
    //      goal.trajectory.points[ind].velocities[j] = 0.0;
    //    }


    //    for (size_t ind = 1; ind < NUM_SEGMENTS; ind++) {
    //      goal.trajectory.points[ind].time_from_start = goal.trajectory.points[ind].time_from_start * 2;
    //    }

    //we are done; return the goal
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
  ros::init(argc, argv, "robot_driver");

  RobotArm arm;
  // Start the trajectory
  arm.startTrajectory(arm.armExtensionTrajectory());
  // Wait for trajectory completion
  while (!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }
}

