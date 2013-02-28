#!/usr/bin/env python

import roslib; roslib.load_manifest('kurtana_calibration')
import rospy

import tf
import ar_pose.msg

def update_kinect(msg):
	try:
		(trans, rot)= listener.lookupTransform('/katana_pattern_seen', '/kinect_link', rospy.Time(0))
		rospy.loginfo('publishing /katana_pattern -> /kinect_link')
		broadcaster.sendTransform(trans, rot, rospy.Time.now(), '/kinect_link', '/katana_pattern')
	except tf.Exception, e:
		rospy.loginfo('no transform /katana_pattern_seen -> /kinect_link available:\n%s' % e.message)

if __name__ == '__main__':
	rospy.init_node('kinect_transform')

	listener = tf.TransformListener()
	broadcaster= tf.TransformBroadcaster()

	rospy.Subscriber('/ar_pose_markers', ar_pose.msg.ARMarkers, update_kinect)
	rospy.spin()
