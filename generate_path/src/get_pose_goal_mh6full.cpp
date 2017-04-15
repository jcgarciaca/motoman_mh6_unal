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

void printCurrentPose(moveit::planning_interface::MoveGroup &group){
	geometry_msgs::PoseStamped pose_msg;
	pose_msg = group.getCurrentPose("arm_link_tool0");
	std::cout << "--------------------------------------" << std::endl;
	std::cout << "---------------- Pose ----------------" << std::endl;
	std::cout << "--------------------------------------" << std::endl;
	std::cout << "Position X: " << pose_msg.pose.position.x << std::endl;
	std::cout << "Position Y: " << pose_msg.pose.position.y << std::endl;
	std::cout << "Position Z: " << pose_msg.pose.position.z << std::endl;

	std::cout << "Orientation X: " << pose_msg.pose.orientation.x << std::endl;
	std::cout << "Orientation Y: " << pose_msg.pose.orientation.y << std::endl;
	std::cout << "Orientation Z: " << pose_msg.pose.orientation.z << std::endl;
	std::cout << "Orientation W: " << pose_msg.pose.orientation.w << std::endl;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "get_pose_goal_mh6full");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroup group("mh6");

	printCurrentPose(group);
	ros::waitForShutdown();
	return 0;
}
