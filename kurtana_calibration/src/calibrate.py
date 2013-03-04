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

SAMPLES_REQUIRED= 300
TIMEOUT=20

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
[-0.02, 0.47, 0.22, 0.02, -2.94, 0.29, 0.29],
[ 1.08, 0.49, 0.03, 0.00, -2.97, 0.29, 0.29],
[ 1.13, 0.87, 0.07, 0.06, -2.97, 0.29, 0.29],
[ 0.13, 0.51, 0.07, 0.10, -2.97, 0.29, 0.29],
[-0.65, 0.35, 0.08, 0.00, -2.97, 0.29, 0.29],
[-1.33, 0.61, 0.33, 0.29, -2.97, 0.29, 0.29],
[-0.83, 0.87, 0.16, 0.07, -2.97, 0.29, 0.29],
[-0.09, 1.81,-0.86, 0.38, -2.97, 0.29, 0.29],
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

		self.hop(JointState(
			name= ['katana_motor3_lift_joint', 'katana_motor4_lift_joint', 'katana_motor5_wrist_roll_joint', 'katana_r_finger_joint', 'katana_l_finger_joint'],
			position= [-2.18, -2.02, -2.96, 0.28, 0.28]
		), noreset= True)
		self.hop( JointState(name= ['katana_motor2_lift_joint'], position= [2.16]), noreset= True )
		self.hop( JointState(name= ['katana_motor1_pan_joint'], position= [0.00]), noreset= True )
		self.hop(noreset= True)


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

		if transform.samples >= SAMPLES_REQUIRED:
			rospy.loginfo(t)
			rospy.loginfo('averaged over %d samples, moving on' % SAMPLES_REQUIRED)
			try:
				dance.hop()
			except LastHopReachedException:
				break
		elif rospy.get_time() - transform.last_reset > TIMEOUT:
			if transform.samples > 0:
				rospy.logwarn('found only %d samples, but %d are required. Ignoring position!' % (transform.samples, SAMPLES_REQUIRED) )
			else:
				rospy.logwarn('could not detect marker! Ignoring position!')
			dance.hop()
		r.sleep()
