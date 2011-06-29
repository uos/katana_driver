#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>

int main(int argc, char **argv)
{
  //initialize the ROS node
   ros::init(argc, argv, "pick_and_place_app");
   ros::NodeHandle nh;

   //set service and action names
   const std::string OBJECT_DETECTION_SERVICE_NAME =
     "/object_detection";
   const std::string COLLISION_PROCESSING_SERVICE_NAME =
     "/tabletop_collision_map_processing/tabletop_collision_map_processing";
   const std::string PICKUP_ACTION_NAME =
     "/object_manipulator/object_manipulator_pickup";
   const std::string PLACE_ACTION_NAME =
     "/object_manipulator/object_manipulator_place";

   //create service and action clients
   ros::ServiceClient object_detection_srv;
   ros::ServiceClient collision_processing_srv;
   actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction>
     pickup_client(PICKUP_ACTION_NAME, true);
   actionlib::SimpleActionClient<object_manipulation_msgs::PlaceAction>
     place_client(PLACE_ACTION_NAME, true);

   //wait for detection client
   while ( !ros::service::waitForService(OBJECT_DETECTION_SERVICE_NAME,
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

   //wait for pickup client
   while(!pickup_client.waitForServer(ros::Duration(2.0)) && nh.ok())
   {
     ROS_INFO_STREAM("Waiting for action client " << PICKUP_ACTION_NAME);
   }
   if (!nh.ok()) exit(0);

   //wait for place client
   while(!place_client.waitForServer(ros::Duration(2.0)) && nh.ok())
   {
     ROS_INFO_STREAM("Waiting for action client " << PLACE_ACTION_NAME);
   }
   if (!nh.ok()) exit(0);

   //call the tabletop detection
     ROS_INFO("Calling tabletop detector");
     tabletop_object_detector::TabletopDetection detection_call;
     //we want recognized database objects returned
     //set this to false if you are using the pipeline without the database
     detection_call.request.return_clusters = true;
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

     //call object pickup
     ROS_INFO("Calling the pickup action");
     object_manipulation_msgs::PickupGoal pickup_goal;
     //pass one of the graspable objects returned
     //by the collision map processor
     pickup_goal.target = processing_call.response.graspable_objects.at(0);
     //pass the name that the object has in the collision environment
     //this name was also returned by the collision map processor
     pickup_goal.collision_object_name =
       processing_call.response.collision_object_names.at(0);
     //pass the collision name of the table, also returned by the collision
     //map processor
     pickup_goal.collision_support_surface_name =
       processing_call.response.collision_support_surface_name;
     //pick up the object with the right arm
     pickup_goal.arm_name = "arm";
     //specify the desired distance between pre-grasp and final grasp
     pickup_goal.desired_approach_distance = 0.1;
     pickup_goal.min_approach_distance = 0.05;
     //we will be lifting the object along the "vertical" direction
     //which is along the z axis in the base_link frame
     geometry_msgs::Vector3Stamped direction;
     direction.header.stamp = ros::Time::now();
     direction.header.frame_id = "katana_base_link";
     direction.vector.x = 0;
     direction.vector.y = 0;
     direction.vector.z = 1;
     pickup_goal.lift.direction = direction;
     //request a vertical lift of 10cm after grasping the object
     pickup_goal.lift.desired_distance = 0.1;
     pickup_goal.lift.min_distance = 0.05;
     //do not use tactile-based grasping or tactile-based lift
     pickup_goal.use_reactive_lift = false;
     pickup_goal.use_reactive_execution = false;
     //send the goal
     pickup_client.sendGoal(pickup_goal);
     while (!pickup_client.waitForResult(ros::Duration(10.0)))
     {
       ROS_INFO("Waiting for the pickup action...");
     }
     object_manipulation_msgs::PickupResult pickup_result =
       *(pickup_client.getResult());
     if (pickup_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
     {
       ROS_ERROR("The pickup action has failed with result code %d",
                 pickup_result.manipulation_result.value);
       return -1;
     }

     //remember where we picked the object up from
      geometry_msgs::PoseStamped pickup_location;
      if (processing_call.response.graspable_objects.at(0).type ==
          object_manipulation_msgs::GraspableObject::DATABASE_MODEL)
      {
        //for database recognized objects, the location of the object
        //is encapsulated in GraspableObject the message
        pickup_location =
          processing_call.response.graspable_objects.at(0).model_pose.pose;
      }
      else
      {
        //for unrecognized point clouds, the location of the object
        //is considered to be the origin of the frame that the
        //cluster is in
        pickup_location.header =
          processing_call.response.graspable_objects.at(0).cluster.header;
        //identity pose
        pickup_location.pose.orientation.w = 1;
      }
      //create a place location, offset by 10 cm from the pickup location
      geometry_msgs::PoseStamped place_location = pickup_location;
      place_location.header.stamp = ros::Time::now();
      place_location.pose.position.x += 0.1;

      //put the object down
        ROS_INFO("Calling the place action");
        object_manipulation_msgs::PlaceGoal place_goal;
        //place at the prepared location
        place_goal.place_pose = place_location;
        //the collision names of both the objects and the table
        //same as in the pickup action
        place_goal.collision_object_name =
          processing_call.response.collision_object_names.at(0);
        place_goal.collision_support_surface_name =
          processing_call.response.collision_support_surface_name;
        //information about which grasp was executed on the object,
        //returned by the pickup action
        place_goal.grasp = pickup_result.grasp;
        //use the right rm to place
        place_goal.arm_name = "arm";
        //padding used when determining if the requested place location
        //would bring the object in collision with the environment
        place_goal.place_padding = 0.02;
        //how much the gripper should retreat after placing the object
        place_goal.desired_retreat_distance = 0.1;
        place_goal.min_retreat_distance = 0.05;
        //we will be putting down the object along the "vertical" direction
        //which is along the z axis in the base_link frame
        direction.header.stamp = ros::Time::now();
        direction.header.frame_id = "katana_base_link";
        direction.vector.x = 0;
        direction.vector.y = 0;
        direction.vector.z = -1;
        place_goal.approach.direction = direction;
        //request a vertical put down motion of 10cm before placing the object
        place_goal.approach.desired_distance = 0.1;
        place_goal.approach.min_distance = 0.05;
        //we are not using tactile based placing
        place_goal.use_reactive_place = false;
        //send the goal
        place_client.sendGoal(place_goal);
        while (!place_client.waitForResult(ros::Duration(10.0)))
        {
          ROS_INFO("Waiting for the place action...");
        }
        object_manipulation_msgs::PlaceResult place_result =
          *(place_client.getResult());
        if (place_client.getState() !=
            actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_ERROR("Place failed with error code %d",
                    place_result.manipulation_result.value);
          return -1;
        }

}
