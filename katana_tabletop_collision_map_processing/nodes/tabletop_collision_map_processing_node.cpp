/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Author(s): Matei Ciocarlie

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include "tabletop_collision_map_processing/collision_map_interface.h"
#include "tabletop_collision_map_processing/TabletopCollisionMapProcessing.h"

namespace tabletop_collision_map_processing {

static const std::string COLLISION_SERVICE_NAME = "tabletop_collision_map_processing";

class CollisionMapProcessor
{
  private:
    ros::NodeHandle priv_nh_;
    ros::NodeHandle root_nh_;
    CollisionMapInterface collision_map_;
    ros::ServiceServer processing_srv_;
    tf::TransformListener listener_;

    bool serviceCallback(TabletopCollisionMapProcessing::Request &request,
        TabletopCollisionMapProcessing::Response &response)
    {

      try
      {
        // perform resets, if requested
        if (request.reset_collision_models) collision_map_.resetCollisionModels();
        if (request.reset_attached_models) collision_map_.resetAttachedModels();
        if (request.reset_static_map) collision_map_.resetStaticMap();

        //demo_synchronizer::getClient().sync(2, "Detecting table and objects");
        //demo_synchronizer::getClient().rviz(1, "Narrow stereo textured;Collision models");

        //process the table, if requested
        tabletop_object_detector::Table table = request.detection_result.table;
        if (table.x_min!=0 || table.x_max!=0 || table.y_min!=0 || table.y_max!=0)
        {
          collision_map_.processCollisionGeometryForTable(table, "table");
          response.collision_support_surface_name = "table";
        }

        //each cluster gets its own graspable object, regardless if the object detector
        //thinks it should have been merged with another cluster
        for (size_t c=0; c<request.detection_result.clusters.size(); c++)
        {
          object_manipulation_msgs::GraspableObject object;

          //------------- process the cluster part -------------------
          object.cluster = request.detection_result.clusters[c];
          //convert the result to the desired frame
          if (!request.desired_frame.empty())
          {

            object.cluster.header.stamp = ros::Time(0);
            try
            {
              listener_.transformPointCloud(request.desired_frame, object.cluster, object.cluster);
            }
            catch (tf::TransformException ex)
            {
              ROS_ERROR("Failed to transform cluster to %s frame; exception: %s",
                        request.desired_frame.c_str(), ex.what());
              return false;
            }
            //the desired frame becomes the reference frame of the object
            object.reference_frame_id = request.desired_frame;
          }
          else
          {
            //the cluster reference frame becomes the reference frame of the object
            object.reference_frame_id = object.cluster.header.frame_id;
          }
          //compute the cluster bounding box
          object_manipulation_msgs::ClusterBoundingBox bbox;
          collision_map_.getClusterBoundingBox(object.cluster, bbox.pose_stamped, bbox.dimensions);
          //extend the bbox to the table along its z axis
          if ( !collision_map_.extendBoundingBoxZToTable(request.detection_result.table,
                                                         bbox.pose_stamped, bbox.dimensions) )
          {
            ROS_WARN("Failed to extend bbox to table; using original dimensions");
          }

          //-------------- add the object to the collision map as a bounding box ---------------
          std::string collision_name;
          collision_map_.processCollisionGeometryForBoundingBox(bbox, collision_name);
          //insert its collision name into the list
          response.collision_object_names.push_back(collision_name);


          /*
          //-------------- also provide the recognition information, if any --------------
          //temporary: decide here if the object is recognized or not
          //TODO remove this

          int ri = request.detection_result.cluster_model_indices[c];
          ROS_WARN("1");
          if (!request.detection_result.models[ri].model_list.empty() &&
              request.detection_result.models[ri].model_list[0].confidence < 0.005)
          {
            object.potential_models = request.detection_result.models[ri].model_list;
            //convert the results to the desired frame
            ROS_WARN("2");
            if (!request.desired_frame.empty())
            {
              for (size_t m=0; m<object.potential_models.size(); m++)
              {
                ROS_WARN("%d",m);
                object.potential_models[m].pose.header.stamp = ros::Time(0);
                try
                {
                  listener_.transformPose(request.desired_frame,
                                          object.potential_models[m].pose, object.potential_models[m].pose);
                }
                catch (tf::TransformException ex)
                {
                  ROS_ERROR("Failed to transform pose or cluster to %s frame; exception: %s",
                            request.desired_frame.c_str(), ex.what());
                  return false;
                }
              }
            }

          }
          */
          //----------------- add the object to the list -------------------------
          response.graspable_objects.push_back(object);
        }

        if (request.take_static_collision_map)
        {
          //demo_synchronizer::getClient().rviz(1, "Narrow stereo textured;Collision models;Tilt scan");
          collision_map_.takeStaticMap();
          //demo_synchronizer::getClient().rviz(1, "Narrow stereo textured;Collision models;Collision map");
        }
        return true;
       }
      catch (CollisionMapException &ex)
      {
        ROS_ERROR("Collision map processing error; exception: %s", ex.what());
        return false;
      }
    }

  public:
    CollisionMapProcessor() : priv_nh_("~"), root_nh_("")
  {
    //wait forever for connections
    collision_map_.connectionsEstablished(ros::Duration(-1.0));
    //advertise service
    processing_srv_ = priv_nh_.advertiseService(COLLISION_SERVICE_NAME,
        &CollisionMapProcessor::serviceCallback, this);
  }

    ~CollisionMapProcessor() {}
};

} //namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tabletop_collision_map_processing_node");
  tabletop_collision_map_processing::CollisionMapProcessor node;

  ros::spin();
  return 0;
}
