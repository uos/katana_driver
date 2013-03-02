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
[-0.01794416637206675, 0.46678736756758066, 0.2247587731583578, 0.015108006337576363, -2.93629830032396, 0.288538653240145, 0.288538653240145],
[1.0813678254582992, 0.48696479679906823, 0.029335409388473455, 0.001977130793274995, -2.9740955869374623, 0.288538653240145, 0.288538653240145],
[1.1280008414100227, 0.8737098829868617, 0.06969026785144905, 0.06125014843717658, -2.974218305400493, 0.288538653240145, 0.288538653240145],
[0.12858167848676993, 0.5102515347973753, 0.0707487559422808, 0.1001519012179557, -2.974218305400493, 0.288538653240145, 0.288538653240145],
[-0.6489625032767043, 0.3529998978031257, 0.07961359370300025, 0.002099849256305575, -2.974218305400493, 0.288538653240145, 0.288538653240145],
[-1.3311544392652064, 0.6062431735347151, 0.32557976381011855, 0.288647460433344, -2.974218305400493, 0.288538653240145, 0.288538653240145],
[-0.8296040808581173, 0.8704021077030113, 0.1582063344472866, 0.07057675162752108, -2.9740955869374623, 0.288538653240145, 0.288538653240145],
[-0.0875255349105597, 1.8144411737139265, -0.8603238309559407, 0.3827725215780067, -2.968573256101074, 0.288538653240145, 0.288538653240145],
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
	dance.setup()
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
