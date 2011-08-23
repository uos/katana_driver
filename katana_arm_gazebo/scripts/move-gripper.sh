#!/bin/sh
rostopic pub -1 /katana_r_finger_position_controller/command std_msgs/Float64 0.3 &
rostopic pub -1 /katana_l_finger_position_controller/command std_msgs/Float64 0.3 &

sleep 10

rostopic pub -1 /katana_r_finger_position_controller/command std_msgs/Float64 -- -0.4 &
rostopic pub -1 /katana_l_finger_position_controller/command std_msgs/Float64 -- -0.4 &

