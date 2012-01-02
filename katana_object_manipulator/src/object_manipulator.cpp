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

#include "object_manipulator/object_manipulator.h"

#include <algorithm>

//#include <demo_synchronizer/synchronizer_client.h>

#include "object_manipulator/grasp_execution/grasp_executor_with_approach.h"
#include "object_manipulator/grasp_execution/reactive_grasp_executor.h"
#include "object_manipulator/place_execution/place_executor.h"
#include "object_manipulator/tools/grasp_marker_publisher.h"
#include "object_manipulator/tools/exceptions.h"

using object_manipulation_msgs::GraspableObject;
using object_manipulation_msgs::PickupGoal;
using object_manipulation_msgs::PickupResult;
using object_manipulation_msgs::PickupFeedback;
using object_manipulation_msgs::PlaceGoal;
using object_manipulation_msgs::PlaceResult;
using object_manipulation_msgs::PlaceFeedback;
using object_manipulation_msgs::ManipulationResult;
using object_manipulation_msgs::GraspResult;
using object_manipulation_msgs::PlaceLocationResult;

namespace object_manipulator {

ObjectManipulator::ObjectManipulator() :
  priv_nh_("~"),
  root_nh_(""),
  grasp_planning_services_("", "", false),
  marker_pub_(NULL)
{
  bool publish_markers = true;
  if (publish_markers)
  {
    marker_pub_ = new GraspMarkerPublisher();
  }

  grasp_executor_with_approach_ = new GraspExecutorWithApproach(marker_pub_);
  reactive_grasp_executor_ = new ReactiveGraspExecutor(marker_pub_);
  place_executor_ = new PlaceExecutor(marker_pub_);
  reactive_place_executor_ = new ReactivePlaceExecutor(marker_pub_);

  priv_nh_.param<std::string>("default_cluster_planner", default_cluster_planner_, "default_cluster_planner");
  priv_nh_.param<std::string>("default_database_planner", default_database_planner_, "default_database_planner");
  priv_nh_.param<std::string>("default_probabilistic_planner", default_probabilistic_planner_,
			      "default_probabilistic_planner");
  priv_nh_.param<bool>("randomize_grasps", randomize_grasps_, false);

  ROS_INFO("Object manipulator ready. Default cluster planner: %s. Default database planner: %s.",
	   default_cluster_planner_.c_str(), default_database_planner_.c_str());
}

ObjectManipulator::~ObjectManipulator()
{
  delete marker_pub_;
  delete grasp_executor_with_approach_;
  delete reactive_grasp_executor_;
  delete place_executor_;
  delete reactive_place_executor_;
}

void ObjectManipulator::pickup(const PickupGoal::ConstPtr &pickup_goal,
			       actionlib::SimpleActionServer<object_manipulation_msgs::PickupAction> *action_server)
{
  //the result that will be returned
  PickupResult result;
  PickupFeedback feedback;

  //we are making some assumptions here. We are assuming that the frame of the cluster is the
  //cannonical frame of the system, so here we check that the frames of all recognitions
  //agree with that.
  //TODO find a general solution for this problem
  if ( pickup_goal->target.cluster.points.size() > 0)
  {
    for (size_t i=0; i<pickup_goal->target.potential_models.size(); i++)
    {
      if ( pickup_goal->target.potential_models[i].pose.header.frame_id != pickup_goal->target.cluster.header.frame_id)
      {
        ROS_ERROR("Target object recognition result(s) not in the same frame as the cluster");
        result.manipulation_result.value = ManipulationResult::ERROR;
        action_server->setAborted(result);
        return;
      }
    }
  }

  //populate a list of grasps to be tried
  std::vector<object_manipulation_msgs::Grasp> grasps;
  if (!pickup_goal->desired_grasps.empty())
  {
    //use the requested grasps, if any
    grasps = pickup_goal->desired_grasps;
  }
  else
  {
    //decide which grasp planner to call depending on the type of object
    std::string planner_service;
    //option that directly calls low-level planners
    if (!pickup_goal->target.potential_models.empty()) planner_service = default_database_planner_;
    else planner_service = default_cluster_planner_;

    //probabilistic planner
    //planner_service = default_probabilistic_planner_;

    //call the planner and save the list of grasps
    object_manipulation_msgs::GraspPlanning srv;
    srv.request.arm_name = pickup_goal->arm_name;
    srv.request.target = pickup_goal->target;
    srv.request.collision_object_name = pickup_goal->collision_object_name;
    srv.request.collision_support_surface_name = pickup_goal->collision_support_surface_name;
    try
    {
      if (!grasp_planning_services_.client(planner_service).call(srv))
      {
	ROS_ERROR("Object manipulator failed to call planner at %s", planner_service.c_str());
	result.manipulation_result.value = ManipulationResult::ERROR;
	action_server->setAborted(result);
	return;
      }
      if (srv.response.error_code.value != srv.response.error_code.SUCCESS)
      {
	ROS_ERROR("Object manipulator: grasp planner failed with error code %d", srv.response.error_code.value);
	result.manipulation_result.value = ManipulationResult::ERROR;
	action_server->setAborted(result);
	return;
      }
      grasps = srv.response.grasps;
    }
    catch (ServiceNotFoundException &ex)
    {
      ROS_ERROR("Planning service not found");
      result.manipulation_result.value = ManipulationResult::ERROR;
      action_server->setAborted(result);
      return;
    }
  }
  feedback.total_grasps = grasps.size();
  feedback.current_grasp = 0;
  action_server->publishFeedback(feedback);

  //decide which grasp executor will be used
  GraspExecutor *executor;
  if (pickup_goal->use_reactive_execution) executor = reactive_grasp_executor_;
  else executor = grasp_executor_with_approach_;

  //demo_synchronizer::getClient().sync(2, "Attempting to grasp an object.");
  //demo_synchronizer::getClient().rviz(1, "Collision models;Grasp execution;Interpolated IK");

  if (randomize_grasps_)
  {
    ROS_INFO("Randomizing grasps");
    std::random_shuffle(grasps.begin(), grasps.end());
  }

  //try the grasps in the list until one succeeds
  try
  {
    for (size_t i=0; i<grasps.size(); i++)
    {
      if (action_server->isPreemptRequested())
      {
	action_server->setPreempted();
	return;
      }
      feedback.current_grasp = i+1;
      action_server->publishFeedback(feedback);
      GraspResult grasp_result = executor->checkAndExecuteGrasp(*pickup_goal, grasps[i]);
      ROS_DEBUG("Grasp result code: %d; continuation: %d", grasp_result.result_code, grasp_result.continuation_possible);
      result.attempted_grasps.push_back(grasps[i]);
      result.attempted_grasp_results.push_back(grasp_result);
      if (grasp_result.result_code == GraspResult::SUCCESS)
      {
	//demo_synchronizer::getClient().sync(2, "Object grasped - ready to move.");
	//demo_synchronizer::getClient().rviz(1, "Collision models;Planning goal;Collision map;Environment contacts");
	result.manipulation_result.value = ManipulationResult::SUCCESS;
	result.grasp = grasps[i];
	action_server->setSucceeded(result);
	return;
      }
      if (!grasp_result.continuation_possible)
      {
	result.grasp = grasps[i];
	if (grasp_result.result_code == GraspResult::LIFT_FAILED)
	  result.manipulation_result.value = ManipulationResult::LIFT_FAILED;
	else
	  result.manipulation_result.value = ManipulationResult::FAILED;
	action_server->setAborted(result);
	return;
      }
    }
    //all the grasps have been deemed unfeasible
    result.manipulation_result.value = ManipulationResult::UNFEASIBLE;
    action_server->setAborted(result);
    return;
  }
  catch (MoveArmStuckException &ex)
  {
    ROS_ERROR("Grasp aborted because move_arm is stuck");
    result.manipulation_result.value = ManipulationResult::ARM_MOVEMENT_PREVENTED;
    action_server->setAborted(result);
    return;
  }
  catch (GraspException &ex)
  {
    ROS_ERROR("Grasp error; exception: %s", ex.what());
    result.manipulation_result.value = ManipulationResult::ERROR;
    action_server->setAborted(result);
    return;
  }
}

void ObjectManipulator::place(const object_manipulation_msgs::PlaceGoal::ConstPtr &place_goal,
			      actionlib::SimpleActionServer<object_manipulation_msgs::PlaceAction> *action_server)
{
  PlaceResult result;
  PlaceFeedback feedback;
  PlaceExecutor *executor;
  if (place_goal->use_reactive_place)
  {
    executor = reactive_place_executor_;
  }
  else
  {
    executor = place_executor_;
  }

  feedback.total_locations = place_goal->place_locations.size();
  feedback.current_location = 0;
  action_server->publishFeedback(feedback);

  try
  {
    for (size_t i=0; i<place_goal->place_locations.size(); i++)
    {
      if (action_server->isPreemptRequested())
      {
	action_server->setPreempted();
	return;
      }
      feedback.current_location = i+1;
      action_server->publishFeedback(feedback);
      geometry_msgs::PoseStamped place_location = place_goal->place_locations[i];
      PlaceLocationResult location_result = executor->place(*place_goal, place_location);
      ROS_DEBUG("Place location result code: %d; continuation: %d", location_result.result_code,
	       location_result.continuation_possible);
      result.attempted_locations.push_back(place_goal->place_locations[i]);
      result.attempted_location_results.push_back(location_result);
      if (location_result.result_code == PlaceLocationResult::SUCCESS)
      {
	result.place_location = place_goal->place_locations[i];
	result.manipulation_result.value = ManipulationResult::SUCCESS;
	action_server->setSucceeded(result);
	return;
      }
      if (!location_result.continuation_possible)
      {
	result.place_location = place_goal->place_locations[i];
	if (location_result.result_code == PlaceLocationResult::RETREAT_FAILED)
	  result.manipulation_result.value = ManipulationResult::RETREAT_FAILED;
	else
	  result.manipulation_result.value = ManipulationResult::FAILED;
	action_server->setAborted(result);
	return;
      }
    }
    //all the place locations have been deemed unfeasible
    result.manipulation_result.value = ManipulationResult::UNFEASIBLE;
    action_server->setAborted(result);
    return;
  }
  catch (MoveArmStuckException &ex)
  {
    ROS_ERROR("Place aborted because move_arm is stuck");
    result.manipulation_result.value = ManipulationResult::ARM_MOVEMENT_PREVENTED;
    action_server->setAborted(result);
    return;
  }
  catch (GraspException &ex)
  {
    ROS_ERROR("Place error; exception: %s", ex.what());
    result.manipulation_result.value = ManipulationResult::ERROR;
    action_server->setAborted(result);
    return;
  }
}

}
