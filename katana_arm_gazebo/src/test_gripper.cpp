#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper{
private:
  GripperClient* gripper_client_;  

public:
  //Action client initialization
  Gripper(){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);
    
    //wait for the gripper action server to come up 
    while(!gripper_client_->waitForServer(ros::Duration(30.0))){
      ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
    }
  }

  ~Gripper(){
    delete gripper_client_;
  }

  //Open the gripper
  void open(){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.07;
    open.command.max_effort = 100.0;  // use -1 to not limit effort (negative)
    
    ROS_INFO("Sending open goal");
    /*
    gripper_client_->sendGoal(open);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    */
    if (gripper_client_->sendGoalAndWait(open,ros::Duration(10.0),ros::Duration(5.0)) == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper opened!");
    else
      ROS_INFO("The gripper failed to open.");
  }

  //Close the gripper
  void close(){
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.0;
    squeeze.command.max_effort = 50.0;  // Close gently
    
    ROS_INFO("Sending squeeze goal");
    /*
    gripper_client_->sendGoal(squeeze);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    */
    gripper_client_->sendGoal(squeeze);
    if (gripper_client_->waitForResult(ros::Duration(10.0)))
      ROS_INFO("The gripper closed!");
    else
      ROS_INFO("The gripper failed to close.");
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_gripper");

  int param = -1;
  if (argc > 1)
    param = atoi(argv[1]);

  Gripper gripper;

  switch (param)
  {
    case 0:
      for (int i=0; i < 100; i++)
      {
        gripper.open();
        gripper.close();
      }
      break;
    case 1:
      gripper.open();
      break;
    case 2:
      gripper.close();
      break;
    default:
      gripper.open();
      break;
  }

  return 0;
}

