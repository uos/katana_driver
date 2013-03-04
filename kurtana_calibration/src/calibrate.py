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

JOINTS=['katana_motor1_pan_joint', 'katana_motor2_lift_joint', 'katana_motor3_lift_joint', 'katana_motor4_lift_joint', 'katana_motor5_wrist_roll_joint', 'katana_r_finger_joint', 'katana_l_finger_joint']

class Dance:
	def __init__(self):
		self.poses= [
[-0.02, 0.47, 0.22, 0.02, -2.94, 0.29, 0.29],
[ 1.08, 0.49, 0.03, 0.00, -2.97, 0.29, 0.29],
[ 1.13, 0.87, 0.07, 0.06, -2.97, 0.29, 0.29],
[ 0.13, 0.51, 0.07, 0.10, -2.97, 0.29, 0.29],
[-0.65, 0.35, 0.08, 0.00, -2.97, 0.29, 0.29],
[-1.33, 0.61, 0.33, 0.29, -2.97, 0.29, 0.29],
[-0.83, 0.87, 0.16, 0.07, -2.97, 0.29, 0.29],
[-0.09, 1.81,-0.86, 0.38, -2.97, 0.29, 0.29],
#[-2.9633099975755286, 2.1502465205304233, -2.1675566231336343, -1.9598002192179138, -2.9318804356548496, 0.288538653240145, 0.288538653240145]
		]
		self.i= 0
		self.hopping= False
		self.client= SimpleActionClient('katana_arm_controller/joint_movement_action', JointMovementAction)
		self.client.wait_for_server()
		# rospy.loginfo('dancer connected')

	def hop(self, pose=0, noreset= False):
		if pose == 0:
			self.i= (self.i+1) % len(self.poses)
			goal= JointMovementGoal( jointGoal= JointState(name=JOINTS, position=self.poses[self.i]) )
		else:
			goal= JointMovementGoal( jointGoal= JointState(name=JOINTS, position=pose) )

		self.hopping= True
		self.client.send_goal(goal)
		self.client.wait_for_result()
		self.hopping= False
		if not noreset:
			transform.reset()
		return self.client.get_result()
	def setup(self):
		self.hop([-2.9633099975755286, 2.1502465205304233, -2.1675566231336343, -1.9598002192179138, -2.9318804356548496, 0.288538653240145, 0.288538653240145],
			noreset= True)
		self.hop([0.07495371014228791, 2.0798570624900856, -2.2076468595739014, -1.9262980788104915, -2.9904171425205655, 0.288538653240145, 0.288538653240145],
			noreset= True)
		self.hop([0.00684496316016503, 1.1636033088635143, -0.1504752550416364, -0.2234566857943987, -2.9902944240575344, 0.288538653240145, 0.288538653240145],
			noreset= True)

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
		if len(msg.markers) > 0:
			if dance.hopping:
				self.reset()
			try:
				self.listener.waitForTransform('/katana_pattern_seen', '/kinect_link', msg.header.stamp, rospy.Duration(2.0))
				transform= self.listener.lookupTransform('/katana_pattern_seen', '/kinect_link', msg.header.stamp)
				self.addTransform(transform)
				# rospy.loginfo('new transform added')
			except tf.Exception, e:
				rospy.loginfo('no transform /katana_pattern_seen -> /kinect_link available')


if __name__ == '__main__':
	rospy.init_node('kinect_transform')

	dance= Dance()
	transform= TransformBuffer()

	broadcaster= tf.TransformBroadcaster()

	r= rospy.Rate(10)
	dance.setup()
	while not rospy.is_shutdown():
		try:
			t= transform.getTransform()
			broadcaster.sendTransform(t[0], t[1], rospy.Time.now(), '/kinect_link', '/katana_pattern')
		except Exception, e:
			# ignore failing getTransforms
			1

		if transform.samples >= SAMPLES_REQUIRED:
			rospy.loginfo(t)
			rospy.loginfo('averaged over %d samples, moving on' % SAMPLES_REQUIRED)
			dance.hop()
		elif rospy.get_time() - transform.last_reset > TIMEOUT:
			if transform.samples > 0:
				rospy.logwarn('found only %d samples, but %d are required. Ignoring position!' % (transform.samples, SAMPLES_REQUIRED) )
			else:
				rospy.logwarn('could not detect marker! Ignoring position!')
			dance.hop()
		r.sleep()
