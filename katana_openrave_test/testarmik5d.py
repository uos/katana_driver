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
    req.iktype = 'TranslationDirection5D'
    # req.iktype = 'Translation3D'
    req.filteroptions = orrosplanning.srv.IKRequest.RETURN_CLOSEST_SOLUTION
    while True:
        #randomquat = random.rand(4)
        #randomquat /= linalg.norm(randomquat)

        req.pose_stamped.pose.position.x = 0.16192;
        req.pose_stamped.pose.position.y = 0.161038;
        req.pose_stamped.pose.position.z = 0.517586;
        req.pose_stamped.pose.orientation.x = 0.708434;
        req.pose_stamped.pose.orientation.y = 0.024962;
        req.pose_stamped.pose.orientation.z = 0.704894;
        req.pose_stamped.pose.orientation.w = 0.024962;

        #req.pose_stamped.pose.orientation = geometry_msgs.msg.Quaternion(*list(randomquat))
        #coords = random.rand(3)-0.5
        #req.pose_stamped.pose.position = geometry_msgs.msg.Point(*list(coords))
        res=IKFn(req)
        if res is not None and res.error_code.val == ArmNavigationErrorCodes.SUCCESS:
            print 'found solution!'
            print res
