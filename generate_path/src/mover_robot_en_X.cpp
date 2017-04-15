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

void printPose(geometry_msgs::Pose pose_data);
int calculateIK(moveit_msgs::GetPositionIK &server, ros::ServiceClient &client,
		move_group_interface::MoveGroup &group, sensor_msgs::JointState &joint_state,
		geometry_msgs::Pose &pose);
int generateTrajectory(geometry_msgs::Pose &pose, std::vector<geometry_msgs::Pose> &trajectory);
int executeTrajectory(std::vector<geometry_msgs::Pose> &trajectory,
		move_group_interface::MoveGroup &group, move_group_interface::MoveGroup &groupIK);


int main(int argc, char *argv[]){
 ros::init(argc, argv, "mover_robot_en_X");
 ros::NodeHandle nh;
 ros::AsyncSpinner spinner(1);
 spinner.start();

 move_group_interface::MoveGroup group_IK("arm_on_rail");
 group_IK.setPlannerId("RRTConnectkConfigDefault");

 move_group_interface::MoveGroup group_move("mh6");
 group_move.setPlannerId("RRTConnectkConfigDefault");

 // get current pose
 geometry_msgs::PoseStamped pose_stmp_msg;
 pose_stmp_msg = group_IK.getCurrentPose("arm_link_tool0");
 geometry_msgs::Pose pose_msg = pose_stmp_msg.pose;

 // create trajectory
 std::vector<geometry_msgs::Pose> trajectory;
 generateTrajectory(pose_msg, trajectory);
 
 // print trajecotry
 std::cout << "size: " << trajectory.size() << std::endl;
/*
 for(int i = 0; i < trajectory.size(); i++){
	 std::cout << trajectory[i].position.x << std::endl;
 }
*/
 executeTrajectory(trajectory, group_move, group_IK);

 printPose(pose_msg);

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

int calculateIK(moveit_msgs::GetPositionIK &server, ros::ServiceClient &client,
		move_group_interface::MoveGroup &group, sensor_msgs::JointState &joint_state,
		geometry_msgs::Pose &pose){

 server.request.ik_request.group_name = group.getName();
 robot_state::RobotState start_state(*group.getCurrentState());
 moveit_msgs::RobotState robot_state;
 robot_state::robotStateToRobotStateMsg(start_state, robot_state);
 server.request.ik_request.ik_link_name = "arm_link_tool0";

 // get robot current pose
 server.request.ik_request.pose_stamped = group.getCurrentPose("arm_link_tool0");

 // set desired position
 server.request.ik_request.pose_stamped.pose = pose;

 server.request.ik_request.timeout = ros::Duration(10.0);
 server.request.ik_request.attempts = 0;

 client.call(server);

 std::cout << "--------------------------------------" << std::endl;
 if(server.response.error_code.val == 1){
	std::cout << " IK found !!" << std::endl;
	joint_state = server.response.solution.joint_state;
	for(int i = 0; i < server.response.solution.joint_state.position.size(); i++){
		std::cout << "i: " << joint_state.position[i] << std::endl;
	}
	return 1;
 }else{
	 std::cout << " No IK found" << std::endl;
	 return -1;
 }
}

int generateTrajectory(geometry_msgs::Pose &pose, std::vector<geometry_msgs::Pose> &trajectory){
	geometry_msgs::Pose pose_temp;
	pose_temp = pose;
	int desired_points = 20;

	for(int i = 0; i < desired_points; i++){
		pose.position.x += 0.01;
		trajectory.push_back(pose);
	}
	pose = pose_temp;
	std::cout << "size of trajectory: " << trajectory.size() << std::endl;
	return 1;
}

int executeTrajectory(std::vector<geometry_msgs::Pose> &trajectory,
		move_group_interface::MoveGroup &group, move_group_interface::MoveGroup &groupIK){

	// create server and client
	ros::NodeHandle nh_aux;
	moveit_msgs::GetPositionIK server;
	ros::ServiceClient client = nh_aux.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

	sensor_msgs::JointState joint_state;
	moveit::planning_interface::MoveGroup::Plan my_plan;

	// cada punto de la trayectoria
	for(int i = 0; i < trajectory.size(); i++){
		std::cout << "punto: " << i << std::endl;
		calculateIK(server, client, groupIK, joint_state, trajectory[i]);
		group.setJointValueTarget(joint_state);
		bool success = group.plan(my_plan);
		if(success){
		  std::cout << "Planning successfully" << std::endl;
		  group.move();
		}else
		  std::cout << "Planning failed" << std::endl;
		ros::WallDuration(0.1).sleep();
	}
	return 1;
}
