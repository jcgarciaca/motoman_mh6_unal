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

#include <moveit_msgs/GetPositionIK.h>
#include <moveit/robot_state/conversions.h>

#include <cstdlib>
#include <iostream>
#include <fstream>

void printPose(geometry_msgs::Pose &pose_msg);

int main(int argc, char *argv[]){
	ros::init(argc, argv, "move_positioner");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	move_group_interface::MoveGroup group("station");
	group.setPlannerId("RRTConnectkConfigDefault");

	// get current pose
	geometry_msgs::PoseStamped pose_stmp_msg;
	pose_stmp_msg = group.getCurrentPose("station_link_s1");
	geometry_msgs::Pose pose_msg = pose_stmp_msg.pose;

	pose_msg.orientation.x = 0.258829;
	pose_msg.orientation.w = 0.965923;

	group.setPoseTarget(pose_msg, "station_link_s1");
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);

	if(success){
	   ROS_INFO("Planning successfully");
	   group.move();
	}else{
	   ROS_INFO("Planning failed");
	}

	//printPose(pose_msg);

	ros::waitForShutdown();
	return 0;
}

void printPose(geometry_msgs::Pose &pose_msg){
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
