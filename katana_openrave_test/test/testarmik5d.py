#!/usr/bin/env python
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__license__ = 'Apache License, Version 2.0'

import roslib; roslib.load_manifest('orrosplanning')
import rospy, time
import orrosplanning.srv
import geometry_msgs.msg
import kinematics_msgs.srv
from numpy import *
from motion_planning_msgs.msg import ArmNavigationErrorCodes

if __name__=='__main__':
    rospy.init_node('armik_test')
    rospy.wait_for_service('IK')
    IKFn = rospy.ServiceProxy('IK',orrosplanning.srv.IK)
    req = orrosplanning.srv.IKRequest()
    req.pose_stamped.header.frame_id = 'katana_base_link'
    req.manip_name = 'arm'
    #req.iktype = 'Rotation3D'
    #req.iktype = 'Ray4D'
    req.iktype = 'TranslationDirection5D'
    req.filteroptions = 0
    counter = 0
    success = 0
    while True:
        randomquat = random.rand(4)
        randomquat /= linalg.norm(randomquat)
        req.pose_stamped.pose.orientation = geometry_msgs.msg.Quaternion(*list(randomquat))
        #x = random.rand()*(-0.025-(-0.020))+(-0.020)
        #y = random.rand()*(0.025-(0.020))+(0.020)
        #z = random.rand()*(0.45-(0.30))+(0.30)
        x = random.rand()*(-0.15-(0.15))+(0.15)
        y = random.rand()*(0.15-(-0.15))+(-0.15)
        z = random.rand()*(0.55-(0.15))+(0.15)
        req.pose_stamped.pose.position.x = x
        req.pose_stamped.pose.position.y = y
        req.pose_stamped.pose.position.z = z
        res=IKFn(req)
        counter = counter + 1
        if res is not None and res.error_code.val == ArmNavigationErrorCodes.SUCCESS:
            success = success + 1
            print 'found solution for:'
            print x
            print y
            print z
            print randomquat
            print 'success:'
            print success
            print 'counter:'
            print counter
            print res
