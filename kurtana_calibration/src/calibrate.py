#!/usr/bin/env python

import roslib; roslib.load_manifest('kurtana_calibration')
import rospy

import tf
from tf.transformations import quaternion_slerp

from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from sensor_msgs.msg import JointState
from katana_msgs.msg import JointMovementAction, JointMovementGoal

from actionlib import SimpleActionClient

import ar_pose.msg

JOINTS=['katana_motor1_pan_joint', 'katana_motor2_lift_joint', 'katana_motor3_lift_joint', 'katana_motor4_lift_joint', 'katana_motor5_wrist_roll_joint', 'katana_r_finger_joint', 'katana_l_finger_joint']

class CalibrateException(Exception):
	pass
class NoTransformCachedException(CalibrateException):
	pass
class LastHopReachedException(CalibrateException):
	pass


class Dance:
	"""Move Katana arm around in a given fashion"""

	def __init__(self):
		self.poses= [
[-0.07, 0.21, -0.14, -1.03, -2.96, 0.27, 0.27],
[-1.35, 0.15, -0.25, -0.89, -2.96, 0.27, 0.27],
[-1.3, 0.96, -0.09, 0.33, -2.96, 0.27, 0.27],
[-0.07, 0.92, 0.1, 0.16, -2.96, 0.27, 0.27],
[1.13, 0.93, 0.02, 0.16, -2.96, 0.27, 0.27],
[1.64, 0.05, 0.26, -0.19, -2.96, 0.27, 0.27],
		]
		self.i= -1
		self.hopping= False
		self.client= SimpleActionClient('katana_arm_controller/joint_movement_action', JointMovementAction)
		self.client.wait_for_server()

	def hop(self, pose= None, noreset= False):
		"""Hop to next stance of the dance

		pose - JointState, optional. If given, this pose will be taken instead of the next one
		noreset - Boolean, optional. If True, don't reset the PoseBuffer during movement
		"""
		if pose == None:
			if self.i+1 >= len(self.poses):
				raise LastHopReachedException()
			self.i= (self.i+1) % len(self.poses)
			goal= JointMovementGoal( jointGoal= JointState(name=JOINTS, position=self.poses[self.i]) )
		else:
			goal= JointMovementGoal(jointGoal= pose)

		self.hopping= True
		if not noreset:
			transform.reset()
		self.client.send_goal(goal)
		self.client.wait_for_result()
		self.hopping= False
		if not noreset:
			transform.reset()
		return self.client.get_result()

	def setup(self):
		"""Get as safe as possible to the first stance"""

		self.old_pose= rospy.wait_for_message('/katana_joint_states', JointState, 2.0)

		self.hop(JointState(
			name= ['katana_motor3_lift_joint', 'katana_motor4_lift_joint', 'katana_motor5_wrist_roll_joint', 'katana_r_finger_joint', 'katana_l_finger_joint'],
			position= [-2.18, -2.02, -2.96, 0.28, 0.28]
		), noreset= True)
		self.hop( JointState(name= ['katana_motor2_lift_joint'], position= [2.16]), noreset= True )
		self.hop( JointState(name= ['katana_motor1_pan_joint'], position= [0.00]), noreset= True )
		self.hop(noreset= True)

	def restoreOldPose(self):
		"""This restores the pose of the arm before setup() was called"""
		self.hop( JointState(name= JOINTS, position= [0.00, 2.16, -2.18, -2.02, -2.96, 0.28, 0.28]), noreset= True )
		self.hop( JointState(name= ['katana_motor1_pan_joint'], position= [self.old_pose.position[self.old_pose.name.index('katana_motor1_pan_joint')]]), noreset= True)
		self.hop( JointState(name= ['katana_motor2_lift_joint'], position= [self.old_pose.position[self.old_pose.name.index('katana_motor2_lift_joint')]]), noreset= True)
		self.hop( JointState(name=[
			'katana_motor3_lift_joint', 'katana_motor4_lift_joint', 'katana_motor5_wrist_roll_joint', 'katana_r_finger_joint', 'katana_l_finger_joint'
			],
			position= [
				self.old_pose.position[self.old_pose.name.index('katana_motor3_lift_joint')],
				self.old_pose.position[self.old_pose.name.index('katana_motor4_lift_joint')],
				self.old_pose.position[self.old_pose.name.index('katana_motor5_wrist_roll_joint')],
				self.old_pose.position[self.old_pose.name.index('katana_r_finger_joint')],
				self.old_pose.position[self.old_pose.name.index('katana_l_finger_joint')]
			]
		) )

class TransformBuffer:
	"""Accumulate poses of the camera as average"""

	def __init__(self):
		self.reset()
		self.listener= tf.TransformListener()
		rospy.Subscriber('/ar_pose_markers', ar_pose.msg.ARMarkers, self.update_cb)

	def addTransform(self, transform):
		"""add new transform to current average"""
		self.samples+= 1
		factor= float(self.samples-1)/(self.samples)
		self.translation= tuple(map(lambda x,y: factor*x + (1-factor)*y, self.translation, transform[0]))
		self.rotation= quaternion_slerp(self.rotation, transform[1], 1.0/self.samples)

	def getTransform(self):
		if self.samples == 0:
			raise NoTransformCachedException()
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

				# it's not beautiful, but transformPose expects a PoseStamped object
				pose= PoseStamped( msg.header, Pose(Point(*(transform[0])), Quaternion(*(transform[1]))) )
				pose.header.frame_id= '/katana_pattern'
				base_transform= self.listener.transformPose('/base_link', pose)

				p= base_transform.pose.position
				r= base_transform.pose.orientation
				self.addTransform(((p.x,p.y,p.z),(r.x, r.y, r.z, r.w)))
			except tf.Exception, e:
				rospy.loginfo('no transform /katana_pattern_seen -> /kinect_link available')


if __name__ == '__main__':
	rospy.init_node('kinect_transform')

	samples_required= rospy.get_param('~samples_required')
	timeout= rospy.get_param('~timeout')

	dance= Dance()
	transform= TransformBuffer()

	broadcaster= tf.TransformBroadcaster()

	r= rospy.Rate(10)
	dance.setup()
	while not rospy.is_shutdown():
		try:
			t= transform.getTransform()
			broadcaster.sendTransform(t[0], t[1], rospy.Time.now(), '/kinect_link', '/base_link')
		except NoTransformCachedException:
			pass

		if transform.samples >= samples_required:
			rospy.loginfo(t)
			rospy.loginfo('averaged over %d samples, moving on' % samples_required)
			try:
				dance.hop()
			except LastHopReachedException:
				break
		elif rospy.get_time() - transform.last_reset > timeout:
			if transform.samples > 0:
				rospy.logwarn('found only %d samples, but %d are required. Ignoring position!' % (transform.samples, samples_required) )
			else:
				rospy.logwarn('could not detect marker! Ignoring position!')
			dance.hop()
		r.sleep()
	dance.restoreOldPose()
