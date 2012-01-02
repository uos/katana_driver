#include <ros/ros.h>
#include <tabletop_object_detector/TabletopDetection.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>
#include <object_manipulation_msgs/GraspPlanning.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv)
{
  //initialize the ROS node
  ros::init(argc, argv, "tabletop_perception_test");
  ros::NodeHandle nh;

  //set service and action names
  const std::string OBJECT_DETECTION_SERVICE_NAME = "/object_detection";
  const std::string COLLISION_PROCESSING_SERVICE_NAME =
      "/tabletop_collision_map_processing/tabletop_collision_map_processing";
  const std::string GRASP_PLANNER_SERVICE_NAME = "/openrave_grasp_planner";

  //create service and action clients
  ros::ServiceClient object_detection_srv;
  ros::ServiceClient collision_processing_srv;
  ros::ServiceClient grasp_planner_srv;

  //wait for detection client
  while (!ros::service::waitForService(OBJECT_DETECTION_SERVICE_NAME, ros::Duration(2.0)) && nh.ok())
  {
    ROS_INFO("Waiting for object detection service to come up");
  }
  if (!nh.ok())
    exit(0);
  object_detection_srv = nh.serviceClient<tabletop_object_detector::TabletopDetection> (OBJECT_DETECTION_SERVICE_NAME,
                                                                                        true);

  //wait for collision map processing client
  while (!ros::service::waitForService(COLLISION_PROCESSING_SERVICE_NAME, ros::Duration(2.0)) && nh.ok())
  {
    ROS_INFO("Waiting for collision processing service to come up");
  }
  if (!nh.ok())
    exit(0);
  collision_processing_srv
      = nh.serviceClient<tabletop_collision_map_processing::TabletopCollisionMapProcessing> (
                                                                                             COLLISION_PROCESSING_SERVICE_NAME,
                                                                                             true);

  //wait for grasp planner client
  while (!ros::service::waitForService(GRASP_PLANNER_SERVICE_NAME, ros::Duration(2.0)) && nh.ok())
  {
    ROS_INFO("Waiting for grasp planner service to come up");
  }
  if (!nh.ok())
    exit(0);
  grasp_planner_srv = nh.serviceClient<object_manipulation_msgs::GraspPlanning> (GRASP_PLANNER_SERVICE_NAME,
                                                                                 true);

  //call the tabletop detection
  ROS_INFO("Calling Tabletop Detector");
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
  if (detection_call.response.detection.result != detection_call.response.detection.SUCCESS)
  {
    ROS_ERROR("Tabletop detection returned error code %d",
        detection_call.response.detection.result);
    return -1;
  }
  if (detection_call.response.detection.clusters.empty() && detection_call.response.detection.models.empty())
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
  tabletop_collision_map_processing::TabletopCollisionMapProcessing processing_call;
  //pass the result of the tabletop detection
  processing_call.request.detection_result = detection_call.response.detection;
  //ask for the exising map and collision models to be reset
  processing_call.request.reset_static_map = true;
  processing_call.request.reset_collision_models = true;
  processing_call.request.reset_attached_models = true;
  //ask for a new static collision map to be taken with the laser
  //after the new models are added to the environment
  processing_call.request.take_static_collision_map = false;
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

  ROS_ERROR("und weiter");

  //call the planner and save the list of grasps
  object_manipulation_msgs::GraspPlanning srv;
  srv.request.arm_name = "arm";

  ROS_INFO("taking first object with:");
  ROS_INFO("reference_frame: %s", processing_call.response.graspable_objects.at(0).reference_frame_id.c_str());
  ROS_INFO("cluster_frame: %s", processing_call.response.graspable_objects.at(0).cluster.header.frame_id.c_str());

  srv.request.target = processing_call.response.graspable_objects.at(0);
  //srv.request.collision_object_name = pickup_goal->collision_object_name;
  //srv.request.collision_support_surface_name = processing_call.response.collision_support_surface_name;

  if (!grasp_planner_srv.call(srv))
  {
    ROS_ERROR("Object manipulator failed to call planner at"); // %s"), grasp_planner_srv.c_str());
    // result.manipulation_result.value = ManipulationResult::ERROR;
    // action_server->setAborted(result);
    return 0;
  }

  if (srv.response.error_code.value != srv.response.error_code.SUCCESS)
  {
    ROS_ERROR("Object manipulator: grasp planner failed with error code %d", srv.response.error_code.value);
    // result.manipulation_result.value = ManipulationResult::ERROR;
    //action_server->setAborted(result);
    return 0;
  }


  //object_manipulation_msgs::Grasp grasps[] = srv.response.grasps;
  if (srv.response.error_code.value == srv.response.error_code.SUCCESS)
    {
      ROS_INFO("Calling the GraspPlanner succeded");
    }

  object_manipulation_msgs::Grasp current_grasp = srv.response.grasps.at(0);

  ROS_INFO("Current Grasp at: %f % f % f // %f %f %f %f",
      current_grasp.grasp_pose.position.x,
      current_grasp.grasp_pose.position.y,
      current_grasp.grasp_pose.position.z,
      current_grasp.grasp_pose.orientation.x,
      current_grasp.grasp_pose.orientation.y,
      current_grasp.grasp_pose.orientation.z,
      current_grasp.grasp_pose.orientation.w);

}
