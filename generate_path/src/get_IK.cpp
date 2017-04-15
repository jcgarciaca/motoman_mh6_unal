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
		move_group_interface::MoveGroup &group, sensor_msgs::JointState &joint_state);

int main(int argc, char *argv[]){
 ros::init(argc, argv, "get_IK");
 ros::NodeHandle nh;
 ros::AsyncSpinner spinner(1);
 spinner.start();

 
 // create server and client
 moveit_msgs::GetPositionIK server;
 ros::ServiceClient client = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

 move_group_interface::MoveGroup group("arm_on_rail");
 group.setPlannerId("RRTConnectkConfigDefault");


 robot_state::RobotState start_state(*group.getCurrentState());
 group.setStartState(start_state);

 sensor_msgs::JointState joint_state;

 calculateIK(server, client, group, joint_state);


 // mover el robot
 move_group_interface::MoveGroup group_move("mh6");
 group_move.setPlannerId("RRTConnectkConfigDefault");
 group_move.setJointValueTarget(joint_state);
 moveit::planning_interface::MoveGroup::Plan my_plan;
 bool success = group_move.plan(my_plan);

 if(success){
   std::cout << "Planning successfully" << std::endl;
   group_move.move();
 }else
   std::cout << "Planning failed" << std::endl;


 // get current pose
 geometry_msgs::PoseStamped pose_stmp_msg;
 pose_stmp_msg = group.getCurrentPose("arm_link_tool0");
 geometry_msgs::Pose pose_msg = pose_stmp_msg.pose;

 printPose(pose_msg);

 ros::waitForShutdown();
 return 0;
}

int calculateIK(moveit_msgs::GetPositionIK &server, ros::ServiceClient &client,
		move_group_interface::MoveGroup &group, sensor_msgs::JointState &joint_state){

 server.request.ik_request.group_name = group.getName();
 robot_state::RobotState start_state(*group.getCurrentState());
 moveit_msgs::RobotState robot_state;
 robot_state::robotStateToRobotStateMsg(start_state, robot_state);
 server.request.ik_request.ik_link_name = "arm_link_tool0";

 // get robot current position
 server.request.ik_request.pose_stamped = group.getCurrentPose("arm_link_tool0");

 // set desired position
 server.request.ik_request.pose_stamped.pose.position.x = 0.463408;
 server.request.ik_request.pose_stamped.pose.position.y = 0.905824;
 server.request.ik_request.pose_stamped.pose.position.z = 1.35719;

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
