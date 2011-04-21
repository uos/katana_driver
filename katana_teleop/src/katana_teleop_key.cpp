/*
 * based on teleop_pr2_keyboard
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Kevin Watts

//TODO adjust author and licence, specific format needed?

#include <katana_teleop/katana_teleop_key.h>

namespace katana{


    KatanaTeleopKey::KatanaTeleopKey() :
      action_client("joint_movement_action", true) {

      ROS_INFO("KatanaTeleopKey starting...");
      ros::NodeHandle n_;
      ros::NodeHandle n_private("~");

      ros::Subscriber js_sub = n_.subscribe("joint_states",1000, &KatanaTeleopKey::jointStateCallback, this);

      sensor_msgs::JointState jg;

      action_client.waitForServer();

      ROS_INFO("KatanaTeleopKey initialized...");
    }


void KatanaTeleopKey::jointStateCallback(const sensor_msgs::JointState::ConstPtr& js){

      ROS_INFO("KatanaTeleopKeyboard received a new JointState");

      movement_goal_.name = js->name;
      movement_goal_.position = js->position;
      ROS_INFO("Updating...");
      for(unsigned int i = 0; i < movement_goal_.position.size(); i++)
      {
        ROS_INFO("Joint %d - %s: %f", i, movement_goal_.name[i].c_str(), movement_goal_.position[i]);
      }
}
void KatanaTeleopKey::keyboardLoop(){
    char c;
    bool dirty=true;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);


    int jointIndex = 0;

    ROS_INFO("Reading from keyboard");
    ROS_INFO("---------------------------");
    ROS_INFO("Use 'WS' to increase/decrease the joint position about 0.05 radian");
    ROS_INFO("Use 'AD' to switch to the next/previous joint");
    ROS_INFO("---------------------------");
    ROS_INFO("Currently Active Joint: %s", movement_goal_.name[jointIndex].c_str());


    for(;;)
    {
      // get the next event from the keyboard
      if(read(kfd, &c, 1) < 0)
      {
        perror("read():");
        exit(-1);
      }

      switch(c)
      {
        // Increasing/Decreasing JointPosition
      case KEYCODE_W:
        ROS_INFO("modifying w");
        movement_goal_.position[jointIndex] += 0.05;
        dirty = true;
        break;
      case KEYCODE_S:
        ROS_INFO("modifying s");
        movement_goal_.position[jointIndex] -= 0.05;
        dirty = true;
        break;
        // Switching active Joint
      case KEYCODE_D:
        jointIndex = (jointIndex+1)%movement_goal_.name.size();
        ROS_INFO("Currently Active Joint: %s", movement_goal_.name[jointIndex].c_str());
        dirty = true;
        break;

      case KEYCODE_A:
        jointIndex--;
        ROS_INFO("Currently Active Joint: %s", movement_goal_.name[jointIndex].c_str());
        dirty = true;
        break;

      }

      katana::JointMovementGoal goal;

      goal.jointGoal = movement_goal_;

      if (dirty == true)
      {
        ROS_WARN("sending new JointMovementActionGoal");
        action_client.sendGoal(goal);
      }
      ros::spinOnce();
    }
  }
} // end namespace "katana"

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "katana_teleop_key");

  katana::KatanaTeleopKey ktk;

  signal(SIGINT,quit);

  ktk.keyboardLoop();

  return(0);
}


