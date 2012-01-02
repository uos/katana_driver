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
#ifndef _MECHANISM_INTERFACE_H_
#define _MECHANISM_INTERFACE_H_

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <tf/transform_listener.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/Vector3.h>

#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>

#include <move_arm_msgs/MoveArmAction.h>

#include <motion_planning_msgs/GetMotionPlan.h>
#include <motion_planning_msgs/convert_messages.h>
#include <motion_planning_msgs/FilterJointTrajectory.h>
#include <motion_planning_msgs/OrderedCollisionOperations.h>

#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

#include <planning_environment_msgs/ContactInformation.h>
#include <planning_environment_msgs/GetStateValidity.h>

#include <interpolated_ik_motion_planner/SetInterpolatedIKMotionPlanParams.h>

#include <mapping_msgs/AttachedCollisionObject.h>

#include <object_manipulation_msgs/ReactiveGraspAction.h>
#include <object_manipulation_msgs/ReactiveLiftAction.h>
#include <object_manipulation_msgs/ReactivePlaceAction.h>
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>
#include <object_manipulation_msgs/GraspStatus.h>

#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_mechanism_msgs/ListControllers.h>

#include "object_manipulator/tools/exceptions.h"
#include "object_manipulator/tools/service_action_wrappers.h"


namespace object_manipulator {

//! A collection of ROS service and action clients needed for grasp execution
class MechanismInterface
{
 private:
  //! The root namespace node handle
  ros::NodeHandle root_nh_;

  //! The private namespace node handle
  ros::NodeHandle priv_nh_;

  //! Transform listener
  tf::TransformListener listener_;

  //! Publisher for attached objects
  ros::Publisher attached_object_pub_;

  //! Returns the joint names for the arm we are using
  std::vector<std::string> getJointNames(std::string arm_name);

  //! Sets the parameters for the interpolated IK server
  void setInterpolatedIKParams(std::string arm_name, int num_steps,
			       int collision_check_resolution, bool start_from_end);

  //! Calls the switch_controllers service
  bool callSwitchControllers(std::vector<std::string> start_controllers, std::vector<std::string> stop_controllers);

  //! Names of the joint and cartesian controllers
  std::string right_cartesian_controller_;
  std::string left_cartesian_controller_;
  std::string right_joint_controller_;
  std::string left_joint_controller_;

 public:

  //----------------------------- Service clients -------------------------------

  //! Client for IK information query that tells us the list of joints to work on
  MultiArmServiceWrapper<kinematics_msgs::GetKinematicSolverInfo> ik_query_client_;

  //! Client for the IK service
  MultiArmServiceWrapper<kinematics_msgs::GetConstraintAwarePositionIK> ik_service_client_;

  //! Client for the FK service
  MultiArmServiceWrapper<kinematics_msgs::GetPositionFK> fk_service_client_;

  //! Client for the Interpolated IK service
  MultiArmServiceWrapper<motion_planning_msgs::GetMotionPlan> interpolated_ik_service_client_;

  //! Client for setting the params of the Interpolated IK service
  MultiArmServiceWrapper<interpolated_ik_motion_planner::SetInterpolatedIKMotionPlanParams>
    interpolated_ik_set_params_client_;

  //! Client for service that queries if a graps is currently active
  MultiArmServiceWrapper<object_manipulation_msgs::GraspStatus> grasp_status_client_;

  //! Client for service that checks state validity
  ServiceWrapper<planning_environment_msgs::GetStateValidity> check_state_validity_client_;

  //! Client for the joint trajectory normalizer service
  ServiceWrapper<motion_planning_msgs::FilterJointTrajectory> joint_trajectory_normalizer_service_;

  //! Client for the switch controller service
  ServiceWrapper<pr2_mechanism_msgs::SwitchController> switch_controller_service_;

  //! Client for the list controllers service
  ServiceWrapper<pr2_mechanism_msgs::ListControllers> list_controllers_service_;

  //------------------------------ Action clients -------------------------------

  //! Action client for reactive grasping
  MultiArmActionWrapper<object_manipulation_msgs::ReactiveGraspAction> reactive_grasp_action_client_;

  //! Action client for reactive lifting
  MultiArmActionWrapper<object_manipulation_msgs::ReactiveLiftAction> reactive_lift_action_client_;

  //! Action client for reactive placing
  MultiArmActionWrapper<object_manipulation_msgs::ReactivePlaceAction> reactive_place_action_client_;

  //! Action client for move_arm
  MultiArmActionWrapper<move_arm_msgs::MoveArmAction> move_arm_action_client_;

  //! Action client for executing a joint trajectory directly, without move arm
  MultiArmActionWrapper<pr2_controllers_msgs::JointTrajectoryAction> traj_action_client_;

  //! Action client for controlling the gripper for executing grasps
  MultiArmActionWrapper<object_manipulation_msgs::GraspHandPostureExecutionAction> hand_posture_client_;

  //! Action client for pointing the head at a target
  ActionWrapper<pr2_controllers_msgs::PointHeadAction> point_head_action_client_;

  //------------------------------ Publishers ----------------------------------

  //! Publishers for Cartesian pose commands
  ros::Publisher right_cartesian_pub_;
  ros::Publisher left_cartesian_pub_;

  //------------------------------ Functionality -------------------------------

  //! Initializes all clients, then calls getIKInformation() and sets default interpolated IK params
  MechanismInterface();

  //------------- IK -------------

  //! Checks if a given arm state is valid; joint_values must contain values for all joints of the arm
  bool checkStateValidity(std::string arm_name, const std::vector<double> &joint_values,
                          const motion_planning_msgs::OrderedCollisionOperations &collision_operations,
                          const std::vector<motion_planning_msgs::LinkPadding> &link_padding);

  //! Checks if a given arm state is valid with no collision operations or link paddings
  bool checkStateValidity(std::string arm_name, const std::vector<double> &joint_values)
  {
    motion_planning_msgs::OrderedCollisionOperations empty;
    std::vector<motion_planning_msgs::LinkPadding> also_empty;
    return checkStateValidity(arm_name, joint_values, empty, also_empty);
  }

  //! Computes an IK solution for a desired pose
  bool getIKForPose(std::string arm_name, const geometry_msgs::PoseStamped &desired_pose,
		    kinematics_msgs::GetConstraintAwarePositionIK::Response& ik_response,
                    const motion_planning_msgs::OrderedCollisionOperations &collision_operations,
                    const std::vector<motion_planning_msgs::LinkPadding> &link_padding);

  //! Computes an FK solution
  bool getFK(std::string arm_name,
	     std::vector<double> positions,
	     geometry_msgs::PoseStamped &pose_stamped);


  //! Computes an interpolated IK path between two poses
  int getInterpolatedIK(std::string arm_name,
			geometry_msgs::PoseStamped start_pose,
			geometry_msgs::Vector3Stamped direction,
			float desired_trajectory_length,
			const std::vector<double> &seed_joint_position,
			const sensor_msgs::JointState &joint_state,
			const motion_planning_msgs::OrderedCollisionOperations &collision_operations,
			const std::vector<motion_planning_msgs::LinkPadding> &link_padding,
			bool reverse_trajectory,
			trajectory_msgs::JointTrajectory &trajectory,
			float &actual_trajectory_length);

  //------ traj controller  ------

  //! Uses the joint trajectory action to execute the desired trajectory
  void attemptTrajectory(std::string arm_name, const trajectory_msgs::JointTrajectory &trajectory, bool unnormalize);

  //! Convenience function that just gets the joint values for the trajectory, and assumes that
  //! we are using the joint returned by the IK server in that order
  void attemptTrajectory(std::string arm_name, const std::vector< std::vector<double> > &positions,
                         bool unnormalize, float time_per_segment);

  //---------- move arm ----------

  //! Uses  move arm to get to the desired set of joint values
  bool attemptMoveArmToGoal(std::string arm_name, const std::vector<double> &desired_joint_values,
                            const motion_planning_msgs::OrderedCollisionOperations &collision_operations,
                            const std::vector<motion_planning_msgs::LinkPadding> &link_padding);

  void modifyMoveArmGoal(move_arm_msgs::MoveArmGoal &move_arm_goal,
                         motion_planning_msgs::ArmNavigationErrorCodes &error_code,
                         std::vector<planning_environment_msgs::ContactInformation> &contact_info_);

  //! Uses move arm to a predefined arm position to the front, where the object can be transferred to the other arm
  bool moveArmToPose(std::string arm_name, const geometry_msgs::PoseStamped &desired_pose,
                     const motion_planning_msgs::OrderedCollisionOperations &collision_operations,
                     const std::vector<motion_planning_msgs::LinkPadding> &link_padding);

  //! Uses move arm to a desired pose while keeping the object in the hand level
  bool moveArmConstrained(std::string arm_name, const geometry_msgs::PoseStamped &commanded_pose,
                          const motion_planning_msgs::OrderedCollisionOperations &collision_operations,
                          const std::vector<motion_planning_msgs::LinkPadding> &link_padding,
                          const motion_planning_msgs::Constraints &path_constraints,
			  const double &redundancy = 0.0,
			  const bool &compute_viable_pose = true);

  //---------- gripper ----------

  //! Requests the hand to pre-grasp, grasp or release
  void handPostureGraspAction(std::string arm_name, const object_manipulation_msgs::Grasp &grasp, int goal);

  //! Queries the hand if a grasp is currently being correctly executed
  bool graspPostureQuery(std::string arm_name);

  //---------- head ----------

  //! Requests the head to point at a specific position
  bool pointHeadAction(const geometry_msgs::PointStamped &target, std::string pointing_frame="/narrow_stereo_optical_frame");

  //------------- tf -------------

  //! Gets the current pose of the gripper in the frame_id specified in the gripper_pose.header
  geometry_msgs::PoseStamped getGripperPose(std::string arm_name, std::string frame_id);

  //! Computes the gripper pose transformed by a certain translation
  geometry_msgs::PoseStamped translateGripperPose(geometry_msgs::Vector3Stamped translation,
						  geometry_msgs::PoseStamped start_pose,
						  std::string arm_id);

  //! Transforms a pose from one frame to another; just passes through to the tf::Listener
  geometry_msgs::PoseStamped transformPose(const std::string target_frame,
					   const geometry_msgs::PoseStamped &stamped_in);

  //! Transforms a cloud from one frame to another; just passes through to the tf::Listener
  void transformPointCloud(std::string target_frame,
			   const sensor_msgs::PointCloud &cloud_in,
			   sensor_msgs::PointCloud &cloud_out);

  //! Given a grasp pose relative to the wrist roll link, returns the current pose of the grasped object
  geometry_msgs::PoseStamped getObjectPoseForGrasp(std::string arm_name,
						   const geometry_msgs::Pose &grasp_pose);

  //! Converts all the internal components of a GraspableObject to the desired frame,
  //! and sets that frame as the reference_frame_id of that object.
  //! Does NOT convert the SceneRegion component which is camera frame by definition.
  void convertGraspableObjectComponentsToFrame(object_manipulation_msgs::GraspableObject &object,
                                               std::string frame_id);

  //! Just a convenience function
  float vectorLen(const geometry_msgs::Vector3 &vec)
  {
    btVector3 v;
    tf::vector3MsgToTF(vec, v);
    return v.length();
  }

  //! Just a convenience function
  geometry_msgs::Vector3 normalize(const geometry_msgs::Vector3 &vec)
  {
    btVector3 v;
    tf::vector3MsgToTF(vec, v);
    v.normalize();
    geometry_msgs::Vector3 vr;
    tf::vector3TFToMsg(v, vr);
    return vr;
  }

  geometry_msgs::Vector3 negate(const geometry_msgs::Vector3 &vec)
  {
    geometry_msgs::Vector3 v;
    v.x = - vec.x;
    v.y = - vec.y;
    v.z = - vec.z;
    return v;
  }

  //------------ controller switching ------------

  //! Check to see if a controller is running
  bool checkController(std::string controller);

  //! Switch one controller for another
  bool switchControllers(std::string start_controller, std::string stop_controller);

  //! Start one controller
  bool startController(std::string controller);

  //! Stop one controller
  bool stopController(std::string controller);

  //! Switch both arms from joint to Cartesian control
  bool switchToCartesian();

  //! Switch both arms from Cartesian to joint control
  bool switchToJoint();

  //------------ Cartesian movement and related helper functions ------------

  //! Set a new desired PoseStamped for the Cartesian controller for arm_name
  void sendCartesianPoseCommand(std::string arm_name, geometry_msgs::PoseStamped desired_pose,
			  double clip_dist, double clip_angle);

  //! Ask the wrist to go incrementally towards a PoseStamped using the Cartesian controller until it gets there
  // (to within tolerances) or times out (returns 0 for error, 1 for got there, -1 for timed out)
  int moveArmToPoseCartesian(std::string arm_name, const geometry_msgs::PoseStamped &desired_pose,
			      ros::Duration timeout, double dist_tol, double angle_tol,
			      double clip_dist, double clip_angle, double timestep);

  //! Translate the gripper by direction using the Cartesian controllers
  int translateGripperCartesian(std::string arm_name, const geometry_msgs::Vector3Stamped &direction,
				ros::Duration timeout, double dist_tol, double angle_tol,
				double clip_dist, double clip_angle, double timestep);

  //! Euclidean distance/angle between two Pose messages
  void poseDists(geometry_msgs::Pose start, geometry_msgs::Pose end, double &pos_dist, double &angle);

  //! Euclidean distance/angle/axis/direction between two Eigen::eigen2_Transform3ds
  void positionAndAngleDist(Eigen::eigen2_Transform3d start, Eigen::eigen2_Transform3d end, double &pos_dist,
						double &angle, Eigen::Vector3d &axis, Eigen::Vector3d &direction);

  //! Clip a desired pose to be no more than clip_dist/clip_angle away from the current gripper pose
  geometry_msgs::PoseStamped clipDesiredPose(std::string arm_name, const geometry_msgs::PoseStamped &desired_pose,
                                             double clip_dist, double clip_angle);

  //------------ misc ------------

  //! Translates the gripper from its current pose to a new one using interpolated ik
  bool translateGripper(std::string arm_name, const geometry_msgs::Vector3Stamped &direction,
			motion_planning_msgs::OrderedCollisionOperations ord,
			const std::vector<motion_planning_msgs::LinkPadding> &link_padding,
			float requested_distance, float min_distance,
			float &actual_distance);

  //! Returns the link padding vector for setting the desired padding to gripper fingertip links
  static std::vector<motion_planning_msgs::LinkPadding> fingertipPadding(std::string arm_name, double pad);

  //! Returns the link padding vector for setting the desired padding to gripper touch links
  static std::vector<motion_planning_msgs::LinkPadding> gripperPadding(std::string arm_name, double pad);

  //------------ attached objects (collision space) ------------

  //! Attaches a given object to the gripper
  void attachObjectToGripper(std::string arm_name, std::string object_collision_name);

  //! Detached the indicated object from the gripper and adds it back in as a non-attached object
  void detachAndAddBackObjectsAttachedToGripper(std::string arm_name, std::string object_collision_name);

};

//! Returns a MechanismInterface singleton
/*! Due to problems with waitForServer, it is not recommended to use multiple instances
  of the MechanismInterface in the same node. This function provides a singleton.

  CAREFUL WITH MULTI-THREADED CODE!!!
*/
inline MechanismInterface& mechInterface()
{
  static MechanismInterface mech_interface;
  return mech_interface;
}

} //namespace object_manipulator

#endif
