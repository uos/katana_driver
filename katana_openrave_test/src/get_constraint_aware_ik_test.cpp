#include <ros/ros.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <sensor_msgs/JointState.h>
#include <ompl/util/RandomNumbers.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <motion_planning_msgs/DisplayTrajectory.h>
#include <motion_planning_msgs/DisplayPath.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_constraint_aware_ik_test");
  ros::NodeHandle rh;
  ros::NodeHandle pn("~");

  ROS_DEBUG("Starting 'get_constraint_aware_ik_test'...");

  std::string ik_service;
  pn.param<std::string> ("ik_service", ik_service, "katana_constraint_aware_kinematics/get_constraint_aware_ik");

  ros::service::waitForService(ik_service);
  ros::service::waitForService("katana_constraint_aware_kinematics/get_ik_solver_info");

//  ros::Publisher pub =
  //   rh.advertise<motion_planning_msgs::DisplayTrajectory> ("planned_robot_pose", 1000);
  ros::ServiceClient ik_client_ =
      rh.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK> (ik_service);
  ros::ServiceClient query_client =
      rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo> ("katana_constraint_aware_kinematics/get_ik_solver_info");
  ROS_ERROR("Connect to the necessary services...");

  // define the query service messages
  ompl::RNG rng = ompl::RNG();
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;

  sensor_msgs::JointState initialState;
  motion_planning_msgs::DisplayTrajectory planned_robot_pose;

  if (query_client.call(request, response))
  {
    for (unsigned int i = 0; i < response.kinematic_solver_info.joint_names.size(); i++)
    {
   //   ROS_INFO("Joint: %d %s", i,
     //     response.kinematic_solver_info.joint_names[i].c_str());
    }

    // publish initial joint state
    initialState.header.stamp = ros::Time::now();
    initialState.name = response.kinematic_solver_info.joint_names;
    initialState.position.resize(response.kinematic_solver_info.joint_names.size());

    for (unsigned int i = 0; i < response.kinematic_solver_info.joint_names.size(); i++)
    {
     // double rand = rng.uniformReal(response.kinematic_solver_info.limits[i].min_position,
      //                              response.kinematic_solver_info.limits[i].max_position);
     // initialState.position[i] = rand;
      // ROS_INFO("Initial Joint Position: %s %f",response.kinematic_solver_info.joint_names[i].c_str(), rand);

    }

    // publish initial position
/*
    planned_robot_pose.trajectory.joint_trajectory.header = initialState.header;
    planned_robot_pose.trajectory.joint_trajectory.joint_names = initialState.name;
    planned_robot_pose.trajectory.joint_trajectory.points.resize(1);
    planned_robot_pose.trajectory.joint_trajectory.points[0].positions.resize(initialState.position.size());

    for (size_t i = 0; i < initialState.position.size(); i++)
    {
      motion_planning_msgs::JointTrajectoryPoint* point = &planned_robot_pose.trajectory.joint_trajectory.points[0];
      point->positions[i] = initialState.position[i];
    }

    pub.publish(planned_robot_pose);
    ROS_DEBUG("Published initial joint state...");
    ros::Duration(1.0).sleep();
*/
  }
  else
  {
    ROS_ERROR("Could not call query service");
    ros::shutdown();
    exit(1);
  }

  // define the service messages
  kinematics_msgs::GetConstraintAwarePositionIK::Request gcapik_req;
  kinematics_msgs::GetConstraintAwarePositionIK::Response gcapik_res;

  sensor_msgs::JointState jointState = initialState;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  do
  {
    gcapik_req.timeout = ros::Duration(5.0);
    gcapik_req.ik_request.ik_link_name = "katana_gripper_tool_frame";

    // generate a random pose in katana's operation space:
    // [-0.12, 0.68] in the z direction and [-0.48, 0.48] in both x and y

    gcapik_req.ik_request.pose_stamped.header.frame_id = "katana_base_link";


     // position should have a solution
/*
    gcapik_req.ik_request.pose_stamped.pose.position.x = 0.16192;
     gcapik_req.ik_request.pose_stamped.pose.position.y = 0.161038;
     gcapik_req.ik_request.pose_stamped.pose.position.z = 0.517586;

     gcapik_req.ik_request.pose_stamped.pose.orientation.x = 0.708434;
     gcapik_req.ik_request.pose_stamped.pose.orientation.y = 0.024962;
     gcapik_req.ik_request.pose_stamped.pose.orientation.z = 0.704894;
     gcapik_req.ik_request.pose_stamped.pose.orientation.w = 0.024962;


*/
    gcapik_req.ik_request.pose_stamped.pose.position.x = rng.uniformReal(-0.48, 0.48);
    gcapik_req.ik_request.pose_stamped.pose.position.y = rng.uniformReal(-0.48, 0.48);
    gcapik_req.ik_request.pose_stamped.pose.position.z = rng.uniformReal(-0.12, 0.68);

    double value[4];

    rng.quaternion(value);

    gcapik_req.ik_request.pose_stamped.pose.orientation.x = value[0];
    gcapik_req.ik_request.pose_stamped.pose.orientation.y = value[1];
    gcapik_req.ik_request.pose_stamped.pose.orientation.z = value[2];
    gcapik_req.ik_request.pose_stamped.pose.orientation.w = value[3];

    // publishing the goal pose as TF for visualising it in rviz

    transform.setOrigin(tf::Vector3(gcapik_req.ik_request.pose_stamped.pose.position.x,
                                    gcapik_req.ik_request.pose_stamped.pose.position.y,
                                    gcapik_req.ik_request.pose_stamped.pose.position.z));

    transform.setRotation(tf::Quaternion(gcapik_req.ik_request.pose_stamped.pose.orientation.x,
                                         gcapik_req.ik_request.pose_stamped.pose.orientation.y,
                                         gcapik_req.ik_request.pose_stamped.pose.orientation.z,
                                         gcapik_req.ik_request.pose_stamped.pose.orientation.w));

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "katana_base_link", "katana_goal_pose"));

    ROS_DEBUG("cartesian goal pose orientation: %f, %f, %f",  gcapik_req.ik_request.pose_stamped.pose.position.x,
                                                              gcapik_req.ik_request.pose_stamped.pose.position.y,
                                                              gcapik_req.ik_request.pose_stamped.pose.position.z);

    ROS_DEBUG("cartesian goal pose orientation: %f, %f, %f, %f", gcapik_req.ik_request.pose_stamped.pose.orientation.x,
                                                                gcapik_req.ik_request.pose_stamped.pose.orientation.y,
                                                                gcapik_req.ik_request.pose_stamped.pose.orientation.z,
                                                                gcapik_req.ik_request.pose_stamped.pose.orientation.w);

    gcapik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());

    gcapik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;

    // let the IK plan from the initial state...
 //   gcapik_req.ik_request.ik_seed_state.joint_state = initialState;

    // or from the middle position of all joints...

    for (unsigned int i = 0; i < response.kinematic_solver_info.joint_names.size(); i++)
    {
      gcapik_req.ik_request.ik_seed_state.joint_state.position[i]
          = (response.kinematic_solver_info.limits[i].min_position
              + response.kinematic_solver_info.limits[i].max_position) / 2.0;
    }


   ros::Rate loop_rate1(100);
   ros::Time end_time = ros::Time::now() + ros::Duration(0.3);

    while (ros::ok() && ros::Time::now() < end_time)
    {
      jointState.header.stamp = ros::Time::now();
    //  planned_robot_pose.trajectory.joint_trajectory.header = jointState.header;
  //    pub.publish(planned_robot_pose);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "katana_base_link", "katana_goal_pose"));

      loop_rate1.sleep();
    }

  //  ROS_WARN("Calling the constraint_aware IK client...");

    bool success = ik_client_.call(gcapik_req, gcapik_res);

    //ROS_WARN("Calling the client succeded...");


    if (!success)
    {
      ROS_ERROR("Could not call service");
      break;
    }

 } while (gcapik_res.error_code.val != gcapik_res.error_code.SUCCESS);


  ROS_INFO("Joint Set Up...");

  for (unsigned int i = 0; i < gcapik_res.solution.joint_state.name.size(); i++)
  {
    ROS_INFO("Joint: %s - %f",gcapik_res.solution.joint_state.name[i].c_str(),gcapik_res.solution.joint_state.position[i]);

  }

  ROS_INFO("...leads to:");


  ROS_INFO("pose position: %f, %f, %f",  gcapik_req.ik_request.pose_stamped.pose.position.x,
                                                               gcapik_req.ik_request.pose_stamped.pose.position.y,
                                                               gcapik_req.ik_request.pose_stamped.pose.position.z);

  ROS_INFO("pose orientation: %f, %f, %f, %f", gcapik_req.ik_request.pose_stamped.pose.orientation.x,
                                                                 gcapik_req.ik_request.pose_stamped.pose.orientation.y,
                                                                 gcapik_req.ik_request.pose_stamped.pose.orientation.z,
                                                                 gcapik_req.ik_request.pose_stamped.pose.orientation.w);

  btScalar roll, pitch, yaw;
   btQuaternion q = btQuaternion(gcapik_req.ik_request.pose_stamped.pose.orientation.x,
                                 gcapik_req.ik_request.pose_stamped.pose.orientation.y,
                                 gcapik_req.ik_request.pose_stamped.pose.orientation.z,
                                 gcapik_req.ik_request.pose_stamped.pose.orientation.w);

   btMatrix3x3(q).getRPY(roll, pitch, yaw);

   ROS_INFO("pose orientation RPY: %f, %f, %f", roll, pitch, yaw);

  ROS_INFO("...");
  // publish goal state

  jointState = gcapik_res.solution.joint_state;
/*
  planned_robot_pose.trajectory.joint_trajectory.header = gcapik_res.solution.joint_state.header;
  planned_robot_pose.trajectory.joint_trajectory.joint_names = gcapik_res.solution.joint_state.name;

  planned_robot_pose.trajectory.joint_trajectory.points[0].positions.clear();
  planned_robot_pose.trajectory.joint_trajectory.points[0].positions.resize(gcapik_res.solution.joint_state.position.size());


  for (size_t i = 0; i <gcapik_res.solution.joint_state.position.size(); i++)
  {
    motion_planning_msgs::JointTrajectoryPoint* point = &planned_robot_pose.trajectory.joint_trajectory.points[0];
    point->positions[i] = gcapik_res.solution.joint_state.position[i];
  }
*/
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ROS_DEBUG("Publish goal state");

    jointState.header.stamp = ros::Time::now();
  //  planned_robot_pose.trajectory.joint_trajectory.header = jointState.header;
   // pub.publish(planned_robot_pose);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "katana_base_link", "katana_goal_pose"));

    loop_rate.sleep();
  }
}

