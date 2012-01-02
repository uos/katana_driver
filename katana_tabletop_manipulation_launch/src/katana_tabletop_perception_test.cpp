#include <ros/ros.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>

int main(int argc, char **argv)
{
  //initialize the ROS node
   ros::init(argc, argv, "tabletop_perception_test");
   ros::NodeHandle nh;

   //set service and action names
   const std::string OBJECT_DETECTION_SERVICE_NAME =
     "/object_detection";
   const std::string COLLISION_PROCESSING_SERVICE_NAME =
     "/tabletop_collision_map_processing/tabletop_collision_map_processing";

   //create service and action clients
   ros::ServiceClient object_detection_srv;
   ros::ServiceClient collision_processing_srv;

   //wait for detection client
   while (!ros::service::waitForService(OBJECT_DETECTION_SERVICE_NAME,
                                         ros::Duration(2.0)) && nh.ok() )
   {
     ROS_INFO("Waiting for object detection service to come up");
   }
   if (!nh.ok()) exit(0);
   object_detection_srv =
     nh.serviceClient<tabletop_object_detector::TabletopDetection>
     (OBJECT_DETECTION_SERVICE_NAME, true);

   //wait for collision map processing client
   while ( !ros::service::waitForService(COLLISION_PROCESSING_SERVICE_NAME,
                                         ros::Duration(2.0)) && nh.ok() )
   {
     ROS_INFO("Waiting for collision processing service to come up");
   }
   if (!nh.ok()) exit(0);
   collision_processing_srv =
     nh.serviceClient
     <tabletop_collision_map_processing::TabletopCollisionMapProcessing>
     (COLLISION_PROCESSING_SERVICE_NAME, true);


   //call the tabletop detection
     ROS_INFO("Calling tabletop detector");
     tabletop_object_detector::TabletopDetection detection_call;
     //we want recognized database objects returned
     //set this to false if you are using the pipeline without the database
     detection_call.request.return_clusters = false;
     //we want the individual object point clouds returned as well
     detection_call.request.return_models = false;
     if (!object_detection_srv.call(detection_call))
     {
       ROS_ERROR("Tabletop detection service failed");
       return -1;
     }
     if (detection_call.response.detection.result !=
         detection_call.response.detection.SUCCESS)
     {
       ROS_ERROR("Tabletop detection returned error code %d",
                 detection_call.response.detection.result);
       return -1;
     }
     if (detection_call.response.detection.clusters.empty() &&
         detection_call.response.detection.models.empty() )
     {
       ROS_ERROR("The tabletop detector detected the table, "
                 "but found no objects");
       return -1;
     }
     else
         {
           ROS_WARN("Tabletop suceeded and found %d unknown clusters and %d known objects, and the table", detection_call.response.detection.clusters.size(), detection_call.response.detection.models.size());
         }

     //call collision map processing
     ROS_INFO("Calling collision map processing");
     tabletop_collision_map_processing::TabletopCollisionMapProcessing
       processing_call;
     //pass the result of the tabletop detection
     processing_call.request.detection_result =
       detection_call.response.detection;
     //ask for the exising map and collision models to be reset
     processing_call.request.reset_static_map = true;
     processing_call.request.reset_collision_models = true;
     processing_call.request.reset_attached_models = true;
     //ask for a new static collision map to be taken with the laser
     //after the new models are added to the environment
     processing_call.request.take_static_collision_map = true;
     //ask for the results to be returned in base link frame
     processing_call.request.desired_frame = "katana_base_link";
     if (!collision_processing_srv.call(processing_call))
     {
       ROS_ERROR("Collision map processing service failed");
       return -1;
     }
     //the collision map processor returns instances of graspable objects
     if (processing_call.response.graspable_objects.empty())
     {
       ROS_ERROR("Collision map processing returned no graspable objects");
       return -1;
     }
     else
      {
        ROS_WARN("Collision map processing succeeded and returned %d graspable objects", processing_call.response.graspable_objects.size() );
      }

}
