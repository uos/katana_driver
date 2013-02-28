#!/usr/bin/env python

from sys import argv
from time import sleep

import roslib; roslib.load_manifest('kurtana_calibration')
import rospy

import actionlib

import katana_msgs.msg
import sensor_msgs.msg

def dance(amplitude, dwell):
	client = actionlib.SimpleActionClient('katana_arm_controller/joint_movement_action', katana_msgs.msg.JointMovementAction)
	rospy.loginfo('Waiting for server');
	client.wait_for_server()
	rospy.loginfo('Connected');
	i= 0
	while i < 4:
		goal= sensor_msgs.msg.JointState(name=['katana_motor1_pan_joint'], position=[amplitude]);
		goal= katana_msgs.msg.JointMovementGoal(jointGoal= goal);
		amplitude*= -1

		client.send_goal(goal)
		client.wait_for_result()
		rospy.loginfo('dwelling for %d seconds' % dwell)
		sleep(dwell)
		i+= 1

	return client.get_result()


if __name__ == '__main__':
	rospy.init_node('dance')
	try:
		g= 1.0
		d= 1.0
		if len(argv) > 1 and float(argv[1]) <= 1.5:
			g= float(argv[1])
		if len(argv) > 2:
			d= float(argv[2])
		if g > 1.5:
			g= 1.0
			rospy.loginfo("invalid amplitude specified, reset to %f" % g)
		dance(g, d)
	except rospy.ROSInterruptException:
		print("program interrupted")
