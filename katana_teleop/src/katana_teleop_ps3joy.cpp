#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>

double max_vel_x, max_rotational_vel;
ros::Publisher vel_pub;
double speed_multiplier;

void ps3joyCallback(const joy::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  if (joy->buttons[8] == 1)   // check for full-speed button
  {
    speed_multiplier = 1.0;
  }
  else if (true)  // check if right analog stick was used to scale speed
  {
    speed_multiplier = 0.5 + (0.5 * joy->axes[3]);  // stick full front -> speed_multiplier = 1.0 , full back -> 0.0
  }
  else  // else use half max speed
  {
    speed_multiplier = 0.5;
  }

  // check if cross is used for steering
  if (joy->buttons[4] || joy->buttons[5] || 
      joy->buttons[6] || joy->buttons[7])
  {
    // note that every button of the cross (axes 4-7) generates 
    // an output in [-1.0, 0.0]
    vel.linear.x = max_vel_x * (joy->axes[4] * -1.0 + joy->axes[6]) * speed_multiplier;
    vel.angular.z = max_rotational_vel * (joy->axes[7] * -1.0 + joy->axes[5]) * speed_multiplier;
  }
  else  // use left analog stick
  {
    vel.linear.x = max_vel_x * joy->axes[1] * speed_multiplier;
    vel.angular.z = max_rotational_vel * joy->axes[0] * speed_multiplier;
  }
  vel_pub.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kurt_teleop_ps3joy");

  ros::NodeHandle nh;
  ros::NodeHandle nh_ns("~");

  nh_ns.param("max_vel_x", max_vel_x, 1.5);
  nh_ns.param("max_rotational_vel", max_rotational_vel, 3.0);

  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber ps3joy_sub = nh.subscribe("joy", 10, ps3joyCallback);

  ros::spin();
}

