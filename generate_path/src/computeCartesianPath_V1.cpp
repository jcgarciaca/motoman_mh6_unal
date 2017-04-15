#include "ros/ros.h"
#include "motoman_msgs/WriteSingleIO.h"
#include "motoman_msgs/ReadSingleIO.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <cstdlib>
#include <iostream>
#include <fstream>

void printPose(geometry_msgs::Pose pose_data);

int main(int argc, char *argv[]){
 ros::init(argc, argv, "plan_trajectory");
 ros::NodeHandle nh;
 ros::AsyncSpinner spinner(1);
 spinner.start();

 move_group_interface::MoveGroup group("arm_on_rail");
 group.setPlannerId("RRTConnectkConfigDefault");

 // get current pose
 geometry_msgs::PoseStamped pose_stmp_msg;
 pose_stmp_msg = group.getCurrentPose("arm_link_tool0");
 geometry_msgs::Pose pose_msg = pose_stmp_msg.pose;

// printPose(pose_msg);

 std::vector<geometry_msgs::Pose> waypoints;
 
 pose_msg.position.x += 0.1;
 waypoints.push_back(pose_msg);

 pose_msg.position.y += 0.1;
 waypoints.push_back(pose_msg);

 pose_msg.position.x -= 0.3;
 waypoints.push_back(pose_msg);

 pose_msg.position.y -= 0.3;
 waypoints.push_back(pose_msg);

 pose_msg.position.z -= 0.2;
 waypoints.push_back(pose_msg);

 moveit_msgs::RobotTrajectory trajectory;
 group.setPlanningTime(10.0);

 double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

 std::cout << "fraction: " << fraction << std::endl;

 moveit::planning_interface::MoveGroup::Plan my_plan;

/*
 bool success = group.plan(my_plan);

// group.execute(my_plan);

 if(success){
   std::cout << "Planning successfully" << std::endl;
   //group.move();
 }else
   std::cout << "Planning failed" << std::endl;

/*
 group.setPoseTarget(pose_msg);

 success = group.plan(my_plan);

 if(success){
   std::cout << "Planning successfully" << std::endl;
   group.move();
 }else
   std::cout << "Planning failed" << std::endl;
*/


 my_plan.trajectory_ = trajectory;
 group.execute(my_plan);
 

 ros::waitForShutdown();
 return 0;
}

void printPose(geometry_msgs::Pose pose_msg){
 std::cout << "--------------------------------------" << std::endl;
 std::cout << "---------------- Pose ----------------" << std::endl;
 std::cout << "--------------------------------------" << std::endl;
 std::cout << "Position X: " << pose_msg.position.x << std::endl;
 std::cout << "Position Y: " << pose_msg.position.y << std::endl;
 std::cout << "Position Z: " << pose_msg.position.z << std::endl;

 std::cout << "Orientation X: " << pose_msg.orientation.x << std::endl;
 std::cout << "Orientation Y: " << pose_msg.orientation.y << std::endl;
 std::cout << "Orientation Z: " << pose_msg.orientation.z << std::endl;
 std::cout << "Orientation W: " << pose_msg.orientation.w << std::endl;
 std::cout << "--------------------------------------" << std::endl;
}
