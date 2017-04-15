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

int main(int argc, char *argv[]){
	ros::init(argc, argv, "pose_arm_on_rail_constraints");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroup group("arm_on_rail");

	std::vector<geometry_msgs::Pose> target_poses;
	geometry_msgs::Pose target_pose;

	target_pose.position.x = 3.79;
	target_pose.position.y = 0.4;
	target_pose.position.z = 1.0;

	target_pose.orientation.x = 0.0;
	target_pose.orientation.y = 0.0;
	target_pose.orientation.z = -1.0;
	target_pose.orientation.w = 0.0;
	target_poses.push_back(target_pose);

	target_pose.position.z = 1.5;
	target_poses.push_back(target_pose);

	target_pose.position.x = 2.8;
	target_poses.push_back(target_pose);

	target_pose.position.y = -0.4;
	target_poses.push_back(target_pose);

	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success;

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
		std::cout << "orientation constraints " << group.getPathConstraints();
		group.setPoseTarget(target_poses[i]);

		success = group.plan(my_plan);
		if(success){
			ROS_INFO("Planning successfully");
			group.move();
		}else{
			ROS_INFO("Planning failed");
		}
		group.clearPathConstraints();
	}

	/*group.setPoseTargets(target_poses);

	std::cout << "getPoseTargets: " << group.getPoseTargets().size() << std::endl;

	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);

	if(success){
		ROS_INFO("Planning successfully");
		group.move();
	}else{
		ROS_INFO("Planning failed");
	}*/


	ros::waitForShutdown();
}
