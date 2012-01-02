#!/usr/bin/env python
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'
__license__ = 'Apache License, Version 2.0'

import roslib; roslib.load_manifest('orrosplanning')
import rospy, time
import object_manipulation_msgs.srv
import object_manipulation_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from numpy import *
import openravepy
import pickle
if __name__=='__main__':
    env=openravepy.Environment()
    body=env.ReadKinBodyXMLFile('data/ketchup.kinbody.xml')
    env.AddKinBody(body,True)
    trimesh=env.Triangulate(body)
    env.Remove(body)
    env.Destroy()

    rospy.init_node('graspplanning_test')
    rospy.wait_for_service('GraspPlanning')
    GraspPlanningFn = rospy.ServiceProxy('GraspPlanning',object_manipulation_msgs.srv.GraspPlanning)
    req = object_manipulation_msgs.srv.GraspPlanningRequest()
    req.arm_name = 'arm'
    #req.target.type = object_manipulation_msgs.msg.GraspableObject.POINT_CLUSTER
    req.target.cluster.header.frame_id = 'katana_base_link'
    offset = [-0.1,-0.3,0.20]
    req.target.cluster.points = [geometry_msgs.msg.Point32(p[0]+offset[0],p[1]+offset[1],p[2]+offset[2]) for p in trimesh.vertices]
    req.target.cluster.channels.append(sensor_msgs.msg.ChannelFloat32(name='indices',values=trimesh.indices.flatten().tolist()))
    req.collision_object_name = ''
    req.collision_support_surface_name = ''
    res=GraspPlanningFn(req)
    pickle.dump(res,open('grasps.pp','w'))
    print res
