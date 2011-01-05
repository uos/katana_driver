/*
 * katana_testpublisher.cpp
 *
 *  Created on: 03.12.2010
 *      Author: martin
 */

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"

#include <sstream>

const double deg2rad = 0.01745329251994329576923690768488;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  // TODO: make this a test?
  ros::init(argc, argv, "katana_testpublisher");
  ros::NodeHandle n;

  ros::Publisher jointStatesPub = n.advertise<sensor_msgs::JointState> ("joint_states", 1000);

  ros::Rate loop_rate(1);

  tf::TransformBroadcaster br;
  tf::Transform transform;

  while (ros::ok())
  {
    /* ************** Publish joint angles ************** */
    sensor_msgs::JointStatePtr jsMsg = boost::make_shared<sensor_msgs::JointState>();

    // The following values were recorded directly from the Katana
    jsMsg->name.push_back("katana_motor1_pan_joint");
    jsMsg->position.push_back(170.0 * deg2rad);
    jsMsg->name.push_back("katana_motor2_lift_joint");
    jsMsg->position.push_back(85.0 * deg2rad);
    jsMsg->name.push_back("katana_motor3_lift_joint");
    jsMsg->position.push_back(190.0 * deg2rad);
    jsMsg->name.push_back("katana_motor4_lift_joint");
    jsMsg->position.push_back(99.0 * deg2rad);
    jsMsg->name.push_back("katana_motor5_wrist_roll_joint");
    jsMsg->position.push_back(110.0 * deg2rad);
    jsMsg->name.push_back("katana_r_finger_joint");
    jsMsg->position.push_back(0.0 * deg2rad);
    jsMsg->name.push_back("katana_l_finger_joint");
    jsMsg->position.push_back(0.0 * deg2rad);

    jsMsg->header.stamp = ros::Time::now();
    jointStatesPub.publish(jsMsg);

    /* ************** Publish corresponding tool frame ************** */
    //    ------------------------------------
    //    X: -168.422
    //    Y: -113.905
    //    Z: 286.677
    //    phi: -0.199944
    //    theta: 1.91903
    //    psi: 3.06725
    //    ------------------------------------

    transform.setOrigin(tf::Vector3(-0.168422, -0.113905, 0.286677));
    transform.setRotation(tf::createQuaternionFromRPY(-0.199944, 1.91903, 3.06725));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "katana_base_frame", "katana_kni_tool_center_point"));

    //    after calibration:
    //    ------------------------------------
    //    X: 157.239
    //    Y: -3.63614
    //    Z: 239.702
    //    phi: 1.52283
    //    theta: 1.94303
    //    psi: 1.4876
    //    ------------------------------------
    transform.setOrigin(tf::Vector3(0.157239, -0.00363614, 0.239702));
    transform.setRotation(tf::createQuaternionFromRPY(1.52283, 1.94303, 1.4876));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "katana_base_frame", "after_calibration"));

    // TODO: test this with real robot before making any modifications


    /* ************** spin & sleep ************** */

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
