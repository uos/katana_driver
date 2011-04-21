#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <joy/Joy.h>

double max_vel_x, max_rotational_vel;
ros::Publisher vel_pub;

void cyborgevoCallback(const joy::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  vel.linear.x = max_vel_x * (joy->axes[2] + 1) / 2 * joy->axes[1];
  vel.angular.z = max_rotational_vel * (joy->axes[2] + 1) / 2 * joy->axes[0];
  vel_pub.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kurt_teleop_cyborgevo");

  ros::NodeHandle nh;
  ros::NodeHandle nh_ns("~");

  nh_ns.param("max_vel_x", max_vel_x, 1.5);
  nh_ns.param("max_rotational_vel", max_rotational_vel, 3.0);

  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber cyborgevo_sub = nh.subscribe("joy", 10, cyborgevoCallback);

  ros::spin();
}

