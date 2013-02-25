#!/bin/sh

gnome-terminal --tab -e roslaunch kurtana_bringup kurtana.launch
rosrun rviz rviz -d ~/ros/uos-ros-pkg/kurtana_robot/kurtana_bringup/rviz/kurtana.vcg &
gnome-terminal --tab -e roslaunch ar_kinect ar_kinect.launch

