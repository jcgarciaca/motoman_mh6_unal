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

	std::vector<geometry_msgs::Pose> target_poses;

	geometry_msgs::Pose target_pose;
	target_pose.position.x = 3.79;
	target_pose.position.y = 0.4;
	target_pose.position.z = 1.0;

	target_pose.orientation.x = 0.0;
	target_pose.orientation.y = 0.0;
	target_pose.orientation.z = -1.0;
	target_pose.orientation.w = 0.0;

	geometry_msgs::Pose group_pose;
	group_pose = getCurrentPose(group, "station_link_s1");

	// add 1st point
	target_poses.push_back(target_pose);

	// add 2nd point
	target_pose.position.z = 1.5;
	target_poses.push_back(target_pose);

	// add 3rd point
	target_pose.position.x = 2.8;
	target_poses.push_back(target_pose);

	// add 4th point
	target_pose.position.y = -0.4;
	target_poses.push_back(target_pose);

	moveit::planning_interface::MoveGroup::Plan robot_plan;

	moveit_msgs::Constraints constraints;
	moveit_msgs::OrientationConstraint ocm;

	for(int i = 0; i < target_poses.size(); i++){
		if(i == 3){
			ocm.link_name = "arm_link_tool0";
			ocm.header.frame_id = "base_rail_link";
			ocm.orientation.z = -1.0;
			ocm.absolute_x_axis_tolerance = 0.1;
			ocm.absolute_y_axis_tolerance = 0.1;
			ocm.absolute_z_axis_tolerance = 0.1;
			ocm.weight = 1.0;
			constraints.orientation_constraints.push_back(ocm);
			group.setPathConstraints(constraints);
		}

		group.setPoseTarget(target_poses[i], "arm_link_tool0");
		group.setPoseTarget(group_pose, "station_link_s1");

		if(group.plan(robot_plan)){
			ROS_INFO("Planning successfully");
			group.move();
			ros::WallDuration(5.0).sleep();
		}else{
			ROS_INFO("Planning failed");
		}
		group.clearPathConstraints();
	}
	ros::waitForShutdown();
}
