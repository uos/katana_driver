#!/usr/bin/env python

import roslib; roslib.load_manifest('kurtana_calibration')
import rospy

import tf
from tf.transformations import quaternion_slerp

from sensor_msgs.msg import JointState
from katana_msgs.msg import JointMovementAction, JointMovementGoal

from actionlib import SimpleActionClient

import ar_pose.msg

SAMPLES_REQUIRED= 300
TIMEOUT=20

class Dance:
	def __init__(self):
		self.poses= [
			JointState(name=['katana_motor1_pan_joint'], position=[+1.0]),
			JointState(name=['katana_motor1_pan_joint'], position=[ 0.0]),
			JointState(name=['katana_motor1_pan_joint'], position=[-1.0])
		]
		self.i= 0
		self.hopping= False
		self.client= SimpleActionClient('katana_arm_controller/joint_movement_action', JointMovementAction)
		self.client.wait_for_server()
		# rospy.loginfo('dancer connected')

	def hop(self):
		self.i= (self.i+1) % len(self.poses)
		self.hopping= True
		self.client.send_goal(JointMovementGoal(jointGoal= self.poses[self.i]))
		self.client.wait_for_result()
		self.hopping= False
		transform.reset()
		return self.client.get_result()


class TransformBuffer:
	def __init__(self):
		self.reset()
		self.listener= tf.TransformListener()
		rospy.Subscriber('/ar_pose_markers', ar_pose.msg.ARMarkers, self.update_cb)

	def addTransform(self, transform):
		self.samples+= 1
		factor= float(self.samples-1)/(self.samples)
		self.translation= tuple(map(lambda x,y: factor*x + (1-factor)*y, self.translation, transform[0]))
		self.rotation= quaternion_slerp(self.rotation, transform[1], 1.0/self.samples)

	def getTransform(self):
		if self.samples == 0:
			raise Exception("no transform available")
		return (self.translation, self.rotation)

	def reset(self):
		self.translation= (0,0,0)
		self.rotation= (1,0,0,0)
		self.samples= 0
		self.last_reset= rospy.get_time()

	def update_cb(self, msg):
		if dance.hopping:
			self.reset()
		try:
			transform= self.listener.lookupTransform('/katana_pattern_seen', '/kinect_link', rospy.Time(0))
			self.addTransform(transform)
			# rospy.loginfo('new transform added')
		except tf.Exception, e:
			1
			# rospy.loginfo('no transform /katana_pattern_seen -> /kinect_link available')



if __name__ == '__main__':
	rospy.init_node('kinect_transform')

	dance= Dance()
	transform= TransformBuffer()

	broadcaster= tf.TransformBroadcaster()

	r= rospy.Rate(10)
	while not rospy.is_shutdown():
		try:
			t= transform.getTransform()
			broadcaster.sendTransform(t[0], t[1], rospy.Time.now(), '/kinect_link', '/katana_pattern')
		except Exception, e:
			# ignore failing getTransforms
			1

		if transform.samples > SAMPLES_REQUIRED:
			rospy.loginfo(t)
			rospy.loginfo('position should be stable, moving on...')
			dance.hop()
		elif rospy.get_time() - transform.last_reset > TIMEOUT:
			rospy.logwarn('could not fixate marker, ignoring position')
			dance.hop()
		r.sleep()
