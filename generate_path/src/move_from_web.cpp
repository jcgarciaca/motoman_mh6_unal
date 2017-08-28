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

#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"

#include <moveit_msgs/ExecuteKnownTrajectory.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>


void printPose(geometry_msgs::Pose &pose_msg);

int calculateIK(moveit_msgs::GetPositionIK &server, ros::ServiceClient &client,
		move_group_interface::MoveGroup &group, geometry_msgs::PoseStamped pose_stmp_msg,
		sensor_msgs::JointState &joint_state, std::vector<double> &group_variable_values);

void messageCallback(const geometry_msgs::Point::Ptr &msg);

int moveRobot(std::vector<double> &group_variable_values_full, move_group_interface::MoveGroup &group_move);

ros::ServiceClient client;

int main(int argc, char *argv[]){

	ros::init(argc, argv, "move_from_web");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	client = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
	ros::Subscriber sub = nh.subscribe("/cmd_pos", 1, messageCallback);

	ros::spin();
	return 0;
}


void messageCallback(const geometry_msgs::Point::Ptr &msg){

	move_group_interface::MoveGroup group("arm_on_rail");
	move_group_interface::MoveGroup group_move("mh6");

	group.setPlannerId("RRTConnectkConfigDefault");
	group_move.setPlannerId("RRTConnectkConfigDefault");

	// get current pose
	geometry_msgs::PoseStamped pose_stmp_msg, target_pose;
	pose_stmp_msg = group.getCurrentPose("arm_link_tool0");
	target_pose = pose_stmp_msg;
	geometry_msgs::Pose pose_msg = pose_stmp_msg.pose;

	printPose(pose_msg);

	std::vector<double> group_variable_values, group_variable_values_full;

	group.getCurrentState() -> copyJointGroupPositions(group.getCurrentState() -> getRobotModel() -> getJointModelGroup(group.getName()),
			group_variable_values);
	group_move.getCurrentState() -> copyJointGroupPositions(group_move.getCurrentState()->getRobotModel()->getJointModelGroup(group_move.getName()),
			group_variable_values_full);

	moveit_msgs::GetPositionIK server;

	sensor_msgs::JointState joint_state;
	std::vector<std::vector<double> > joints_trajectory;

	// update target pose
	target_pose.pose.position.x += msg->x;
	target_pose.pose.position.y += msg->y;
	target_pose.pose.position.z += msg->z;


	std::cout << "target: [" << target_pose.pose.position.x << ", " <<
			target_pose.pose.position.y << ", " <<
			target_pose.pose.position.z << "]" << std::endl;


	if(calculateIK(server, client, group, target_pose, joint_state, group_variable_values_full)){
		ROS_INFO("Entra 1");
		group_variable_values_full[7] = 0.0;

		moveRobot(group_variable_values_full, group_move);

	}else{
		ROS_INFO("IK not found");
	}


}

int moveRobot(std::vector<double> &group_variable_values_full, move_group_interface::MoveGroup &group_move){

	group_move.setJointValueTarget(group_variable_values_full);

	for(int i = 0; i < group_variable_values_full.size(); i++){
		std::cout << "joint " << i << ": " << group_variable_values_full[i] << std::endl;
	}

	moveit::planning_interface::MoveGroup::Plan my_plan;
	ROS_INFO("Entra 1.1");
	bool success = group_move.plan(my_plan);

	ROS_INFO("Entra 2");

	if(success){
		ROS_INFO("Planning successfully");
		group_move.move();
	}else{
		ROS_INFO("Planning failed");
	}
	return 1;
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

int calculateIK(moveit_msgs::GetPositionIK &server, ros::ServiceClient &client,
		move_group_interface::MoveGroup &group, geometry_msgs::PoseStamped pose_stmp_msg,
		sensor_msgs::JointState &joint_state, std::vector<double> &group_variable_values){

	 server.request.ik_request.group_name = group.getName();
	 robot_state::RobotState start_state(*group.getCurrentState());
	 moveit_msgs::RobotState robot_state;
	 robot_state::robotStateToRobotStateMsg(start_state, robot_state);
	 server.request.ik_request.ik_link_name = "arm_link_tool0";

	 // set desired position
	 server.request.ik_request.pose_stamped = pose_stmp_msg;

	 server.request.ik_request.timeout = ros::Duration(10.0);
	 server.request.ik_request.attempts = 0;

	 client.call(server);

	 //std::cout << "--------------------------------------" << std::endl;
	 if(server.response.error_code.val == 1){
		//std::cout << " IK found !!" << std::endl;
		joint_state = server.response.solution.joint_state;
		for(int i = 0; i < group_variable_values.size(); i++){
			group_variable_values[i] = joint_state.position[i];
		}
		return 1;
	 }else{
		 std::cout << " ------- No IK found -------" << std::endl;
		 return -1;
	 }
}
