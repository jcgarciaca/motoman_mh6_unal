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

void printCurrentPose(moveit::planning_interface::MoveGroup &group, std::string end_effector){
	geometry_msgs::PoseStamped pose_msg;
	pose_msg = group.getCurrentPose(end_effector);
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

geometry_msgs::Pose getCurrentPose(moveit::planning_interface::MoveGroup &group, std::string end_effector){
	geometry_msgs::PoseStamped pose_msg;
	geometry_msgs::Pose group_pose;
	pose_msg = group.getCurrentPose(end_effector);

	group_pose = pose_msg.pose;
	return group_pose;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "pose_mh6full_move_orientation_constraints");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroup group("mh6");


	printCurrentPose(group, "arm_link_tool0");
	printCurrentPose(group, "station_link_s1");

	geometry_msgs::Pose target_pose, group_pose;
	target_pose = getCurrentPose(group, "arm_link_tool0");
	group_pose = getCurrentPose(group, "station_link_s1");

	geometry_msgs::Quaternion qt = tf::createQuaternionMsgFromRollPitchYaw(0.4363, 0.0, 0.0);

	// arm and rail position
	target_pose.position.x -= 0.5;

	// station position
	group_pose.orientation.x = qt.x;
	group_pose.orientation.y = qt.y;
	group_pose.orientation.z = qt.z;
	group_pose.orientation.w = qt.w;

	moveit::planning_interface::MoveGroup::Plan robot_plan;
	group.setPoseTarget(target_pose, "arm_link_tool0");
	group.setPoseTarget(group_pose, "station_link_s1");

	if(group.plan(robot_plan)){
		ROS_INFO("Planning successfully");
		group.move();
		ros::WallDuration(5.0).sleep();
	}else{
		ROS_INFO("Planning failed");
	}
	ros::waitForShutdown();
}
