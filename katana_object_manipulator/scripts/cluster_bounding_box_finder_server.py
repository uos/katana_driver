#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# author: Kaijen Hsiao

## @package pyexample
#Offers a service to find the principal directions and bounding box of a
#point cluster

from __future__ import division
import roslib
roslib.load_manifest('object_manipulator')
import rospy
from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxResponse
import object_manipulator.cluster_bounding_box_finder as cluster_bounding_box_finder
import scipy
from object_manipulator.convert_functions import *

##class for the find cluster bounding box service
class ClusterBoundingBoxFinderServer:

    def __init__(self):
        self.cbbf = cluster_bounding_box_finder.ClusterBoundingBoxFinder()

        s = rospy.Service('find_cluster_bounding_box', FindClusterBoundingBox, self.find_cluster_bounding_box_callback)


    ##service callback for the find_cluster_bounding_box service
    def find_cluster_bounding_box_callback(self, req):
        rospy.loginfo("finding the bounding box for a point cluster")

        #use PCA to find the object frame and bounding box dims
        (object_points, object_bounding_box_dims, object_bounding_box, object_to_base_frame, object_to_cluster_frame) = \
                           self.cbbf.find_object_frame_and_bounding_box(req.cluster)

        #problem with finding bounding box
        if object_points == None:
            return FindClusterBoundingBoxResponse(PoseStamped(), Vector3(0,0,0), 1)

        #find the frame at the center of the box, not at the bottom
        transform_mat = scipy.matrix([[1,0,0,0],
                                      [0,1,0,0],
                                      [0,0,1,object_bounding_box_dims[2]/2.],
                                      [0,0,0,1]])
        center_mat = object_to_base_frame * transform_mat

        #convert frame to PoseStamped
        pose = mat_to_pose(center_mat)
        pose_stamped = stamp_pose(pose, self.cbbf.base_frame)

        #transform pose to cluster's frame_id
        transformed_pose_stamped = change_pose_stamped_frame(self.cbbf.tf_listener, pose_stamped, req.cluster.header.frame_id)

        #rospy.loginfo("returning cluster bounding box response:"+str(pose_stamped)+str(object_bounding_box_dims))
        return FindClusterBoundingBoxResponse(transformed_pose_stamped, Vector3(*object_bounding_box_dims), 0)


if __name__ == '__main__':
    rospy.init_node('cluster_bounding_box_finder', anonymous=True)

    cbbfs = ClusterBoundingBoxFinderServer()

    rospy.loginfo("cluster bounding box finder is ready for queries")
    rospy.spin()
