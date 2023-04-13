/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of SRI International nor the names of its
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

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include<ros/ros.h>
#include<string.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;
std::vector<double> coord;
float n[6];
float m[9];


void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "Joint7";
  posture.joint_names[1] = "Joint8";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.0;
  posture.points[0].positions[1] = 0.0;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "Joint7";
  posture.joint_names[1] = "Joint8";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = -0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group, geometry_msgs::Pose box_pose)
{
  // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8. |br|
  // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
  // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
  // transform from `"panda_link8"` to the palm of the end effector.
  grasps[0].grasp_pose.header.frame_id = "Base";
  // tf2::Quaternion orientation;
  // orientation.setRPY(-tau / 4, -tau / 8, -tau / 4);
  grasps[0].grasp_pose.pose.orientation = box_pose.orientation;
  grasps[0].grasp_pose.pose.position.x = box_pose.position.x;
  grasps[0].grasp_pose.pose.position.y = box_pose.position.y;
  grasps[0].grasp_pose.pose.position.z = box_pose.position.z - 0.01;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "Base";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.05;
  grasps[0].pre_grasp_approach.desired_distance = 0.15;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "Base";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.0;
  grasps[0].post_grasp_retreat.desired_distance = 0.0;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick2
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture);
  // END_SUB_TUTORIAL
  ros::Duration(0.5).sleep();
  // BEGIN_SUB_TUTORIAL pick3
  // Set support surface as table1.
  move_group.setSupportSurfaceName("Table");
  // Call pick to pick up the object using the grasps given
  move_group.pick("box", grasps);
  // END_SUB_TUTORIAL
}

void place(moveit::planning_interface::MoveGroupInterface& group, geometry_msgs::Pose box_pose)
{
  // BEGIN_SUB_TUTORIAL place
  // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
  // location in verbose mode." This is a known issue. |br|
  // |br|
  // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
  // a single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "Base";
  // tf2::Quaternion orientation;
  // orientation.setRPY(0, 0, tau / 4);  // A quarter turn about the z-axis
  place_location[0].place_pose.pose.orientation = box_pose.orientation;

  /* For place location, we set the value to the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = box_pose.position.x;
  place_location[0].place_pose.pose.position.y = box_pose.position.y;
  place_location[0].place_pose.pose.position.z = box_pose.position.z;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "Base";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "Base";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.z = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  group.setSupportSurfaceName("Table_1");
  // Call place to place the object using the place locations given.
  group.place("box", place_location);
  // END_SUB_TUTORIAL
}
// (Planning scene interface , x,y,z,orientation,l,w,h)
void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,float pose[3],float orientation[4],float size[3], char planning_frame[50],char name[50])
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = name;
  collision_objects[0].header.frame_id = planning_frame;

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = size[0];
  collision_objects[0].primitives[0].dimensions[1] = size[1];
  collision_objects[0].primitives[0].dimensions[2] = size[2];

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = pose[0];
  collision_objects[0].primitive_poses[0].position.y = pose[1];
  collision_objects[0].primitive_poses[0].position.z = pose[2];
  collision_objects[0].primitive_poses[0].orientation.w = orientation[3];
  collision_objects[0].primitive_poses[0].orientation.x = orientation[0];
  collision_objects[0].primitive_poses[0].orientation.y = orientation[1];
  collision_objects[0].primitive_poses[0].orientation.z = orientation[2];
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void chatterCallback(const std_msgs::Float64MultiArray::ConstPtr& data)
{
  std::vector<double> a = {data->data};
  for (int i=0;i<6;i++)
  { 
    n[i] = data->data[i];
  }
  
}
void chatterCallback_mono(const std_msgs::Float64MultiArray::ConstPtr& data)
{
  std::vector<double> a = {data->data};
  for (int i=0;i<9;i++)
  { m[i] = data->data[i];
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  std_msgs::Float64 state;
  state.data = 1.0;

  ros::Subscriber sub = node_handle.subscribe("/box/Coordinate", 1000, chatterCallback);

  // ros::Subscriber sub_mono = node_handle.subscribe("/box/Coordinate_mono", 1000, chatterCallback_mono);
  ros::Publisher pub = node_handle.advertise<std_msgs::Float64>("/camera/state", 5);

  static const std::string PLANNING_GROUP = "manipulator";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("Base");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  //Starting Stereo Camera Image Processing
  for(int i = 0;i < 100; i++)
  {
    pub.publish(state);
  }
  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  //Adding Table Collision Object
  float pose[] = {0.6,0.0,0.075};
  float orientation_1[] = {0.0,0.0,0.0,1.0};
  float size[] = {0.4, 0.8, 0.15};
  char name[][20] = {"Table","Table_1","box"};
  char Planning_group[] = "Base";
  addCollisionObject(planning_scene_interface,pose,orientation_1,size,Planning_group,name[0]);

  // Adding Table_1 Collision Object
  pose[0] = 0.0;
  pose[1] = 0.6;
  size[0] = 0.8;
  size[1] = 0.4;
  addCollisionObject(planning_scene_interface,pose,orientation_1,size,Planning_group, name[1]);
  ROS_INFO_NAMED("tutorial", "Add an object into the world");

  //Getting to position 1 for Monocular Camera Image Processing
  geometry_msgs::Pose target_pose1;
  tf2Scalar roll = -3.14, pitch = 0, yaw = -1.57;

  tf2::Quaternion orientation;
  orientation.setRPY(roll, pitch, yaw);
  
  ROS_INFO_NAMED("tutorial", "Add an object into the world_2");
  target_pose1.orientation = tf2::toMsg(orientation);
  target_pose1.position.x = n[0] - 0.06;
  target_pose1.position.y = 0;
  target_pose1.position.z = n[2] + 0.4;
  move_group_interface.setPoseTarget(target_pose1);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // // Visualizing plans
  // // ^^^^^^^^^^^^^^^^^
  // // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group_interface.move();

  ros::Subscriber sub_1 = node_handle.subscribe("/box/Coordinate_Mono", 1000, chatterCallback_mono);

  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  ros::Duration(0.5).sleep();

  // Starting Monocular Camera Image Processing
  state.data = 2.0;
  for(int i = 0;i < 100; i++)
  {
    pub.publish(state);
  }

  ros::Duration(2.0).sleep();

  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  roll = m[3]-3.14;
  pitch = m[4];
  yaw = m[5]-1.57;

  orientation.setRPY(roll, pitch, yaw);

  geometry_msgs::Pose box_pose;
  box_pose.orientation = tf2::toMsg(orientation);
  for(int i=0;i<=2;i++)
  {
    pose[i] = m[i];
    size[i] = m[i+6];
  }
  orientation_1[0]  = box_pose.orientation.x;
  orientation_1[1]  = box_pose.orientation.y;
  orientation_1[2]  = box_pose.orientation.z;
  orientation_1[3]  = box_pose.orientation.w;
  addCollisionObject(planning_scene_interface,pose,orientation_1,size,Planning_group,name[2]);

  state.data = 3.0;
  for(int i = 0;i < 100; i++)
  {
    pub.publish(state);
  }

  ROS_INFO_NAMED("tutorial", "Add an object into the world");

  
  target_pose1.orientation = tf2::toMsg(orientation);

  target_pose1.position.x = m[0];
  target_pose1.position.y = m[1];
  target_pose1.position.z = m[2]+0.2;

  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  move_group_interface.setPlanningTime(45.0);

  // addCollisionObjects(planning_scene_interface);

  // Wait a bit for ROS things to initialize
  ros::Duration(1.0).sleep();

  pick(move_group_interface,target_pose1);

  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  ros::Duration(1.0).sleep();

  target_pose1.position.z = m[2]+0.4;
  move_group_interface.setPoseTarget(target_pose1);
  move_group_interface.move();
  

  yaw = yaw+1.57;
  orientation.setRPY(roll, pitch, yaw);
  target_pose1.orientation = tf2::toMsg(orientation);
  float xy = 0;
  xy = target_pose1.position.x;
  target_pose1.position.x = target_pose1.position.y;
  target_pose1.position.y = xy;
  target_pose1.position.z = target_pose1.position.z - 0.4;

  // ros::Duration(0.5).sleep();

  place(move_group_interface,target_pose1);
  
  visual_tools.publishAxisLabeled(target_pose1, "pose2");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  ros::shutdown();
  return 0;
}