#include "ros/ros.h"
#include "motoman_msgs/WriteSingleIO.h"
#include "motoman_msgs/ReadSingleIO.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <cstdlib>
#include <iostream>


void open_gripper(motoman_msgs::WriteSingleIO & srv_write, ros::ServiceClient & client_write,
		motoman_msgs::ReadSingleIO & srv_read, ros::ServiceClient & client_read){
	srv_read.request.address = 20050;
	if(client_read.call(srv_read)){
		if(srv_read.response.value == 1){
			srv_write.request.address = 10032;
			srv_write.request.value = 0;
			if(client_write.call(srv_write)){
				std::cout << "Reset 10032" << std::endl;
			}else{
				ROS_ERROR("Failed to call service write_single_io");
			}
			srv_write.request.address = 10033;
			srv_write.request.value = 1;
			if(client_write.call(srv_write)){
				std::cout << "Set 10033" << std::endl;
			}else{
				ROS_ERROR("Failed to call service write_single_io");
			}
		}else{
			std::cout << "Gripper already open..." << std::endl;
		}
	}else{
		ROS_ERROR("Failed to call service read_single_io");
	}
}

void close_gripper(motoman_msgs::WriteSingleIO & srv_write, ros::ServiceClient & client_write,
		motoman_msgs::ReadSingleIO & srv_read, ros::ServiceClient & client_read){
	srv_read.request.address = 20050;
	if(client_read.call(srv_read)){
		if(srv_read.response.value == 0){
			srv_write.request.address = 10033;
			srv_write.request.value = 0;
			if(client_write.call(srv_write)){
				std::cout << "Reset 10033" << std::endl;
			}else{
				ROS_ERROR("Failed to call service write_single_io");
			}
			srv_write.request.address = 10032;
			srv_write.request.value = 1;
			if(client_write.call(srv_write)){
				std::cout << "Set 10032" << std::endl;
			}else{
				ROS_ERROR("Failed to call service write_single_io");
			}
		}else{
			std::cout << "Gripper already closed..." << std::endl;
		}
	}else{
		ROS_ERROR("Failed to call service read_single_io");
	}
}

void moveHome(move_group_interface::MoveGroup & group, std::vector<double> & group_variable_values){
	for(int i = 0; i < group.getActiveJoints().size(); i++){
		group_variable_values[i] = 0.0;
	}
	group.setJointValueTarget(group_variable_values);
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);

	if(success){
		ROS_INFO("Planning successfully");
		group.move();
	}else{
		ROS_INFO("Planning failed");
	}
}

void movePick(move_group_interface::MoveGroup & group, std::vector<double> & group_variable_values){
	group_variable_values[0] = -1.0;
	group_variable_values[1] = -1.569342732429504;
	group_variable_values[2] = 0.47139230370521545;
	group_variable_values[3] = -0.5396569967269897;
	group_variable_values[4] = 0.0;
	group_variable_values[5] = 1.0110481977462769;
	group_variable_values[6] = 0.0;
	group_variable_values[7] = 2.0;

	group.setJointValueTarget(group_variable_values);
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);

	if(success){
		ROS_INFO("Planning successfully");
		group.move();
	}else{
		ROS_INFO("Planning failed");
	}
}

void movePlace(move_group_interface::MoveGroup & group, std::vector<double> & group_variable_values){
	group_variable_values[0] = -0.695148229598999;
	group_variable_values[1] = -0.8237645030021667;
	group_variable_values[2] = 0.09050678461790085;
	group_variable_values[3] = -0.5564420223236084;
	group_variable_values[4] = 0.0;
	group_variable_values[5] = 0.6469454765319824;
	group_variable_values[6] = -0.7453644275665283;
	group_variable_values[7] = 2.0;

	group.setJointValueTarget(group_variable_values);
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group.plan(my_plan);

	if(success){
		ROS_INFO("Planning successfully");
		group.move();
	}else{
		ROS_INFO("Planning failed");
	}
}

void printPose(move_group_interface::MoveGroup & group){
	geometry_msgs::PoseStamped pose_msg;
	pose_msg = group.getCurrentPose();
	std::cout << "--------------------------------------------" << std::endl;
	std::cout << "---------------- Print Pose ----------------" << std::endl;
	std::cout << "--------------------------------------------" << std::endl;
	std::cout << "Position X: " << pose_msg.pose.position.x << std::endl;
	std::cout << "Position Y: " << pose_msg.pose.position.y << std::endl;
	std::cout << "Position Z: " << pose_msg.pose.position.z << std::endl;
	std::cout << "Orientation X: " << pose_msg.pose.orientation.x << std::endl;
	std::cout << "Orientation Y: " << pose_msg.pose.orientation.y << std::endl;
	std::cout << "Orientation Z: " << pose_msg.pose.orientation.z << std::endl;
	std::cout << "Orientation W: " << pose_msg.pose.orientation.w << std::endl;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "exercise_1");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	move_group_interface::MoveGroup group("mh6");
	std::vector<double> group_variable_values;
	group.getCurrentState() -> copyJointGroupPositions(group.getCurrentState() -> getRobotModel() -> getJointModelGroup(group.getName()), group_variable_values);
	//group.setEndEffector("arm_eef");
	//group.setEndEffectorLink("arm_link_tool0");
	std::cout << "Joint Tolerance: " << group.getGoalJointTolerance() << std::endl;
	std::cout << "Orientation Tolerance: " << group.getGoalOrientationTolerance() << std::endl;
	std::cout << "Position Tolerance: " << group.getGoalPositionTolerance() << std::endl;


	ros::NodeHandle nh;
	motoman_msgs::WriteSingleIO srv_write;
	ros::ServiceClient client_write = nh.serviceClient<motoman_msgs::WriteSingleIO>("write_single_io");

	ros::ServiceClient client_read = nh.serviceClient<motoman_msgs::ReadSingleIO>("read_single_io");
	motoman_msgs::ReadSingleIO srv_read;

	// actions
	open_gripper(srv_write, client_write, srv_read, client_read);
	moveHome(group, group_variable_values);
	printPose(group);
	movePick(group, group_variable_values);
	printPose(group);
	close_gripper(srv_write, client_write, srv_read, client_read);
	movePlace(group, group_variable_values);
	printPose(group);
	open_gripper(srv_write, client_write, srv_read, client_read);
	moveHome(group, group_variable_values);







	/*ros::WallDuration(2.0).sleep();
	close_gripper(srv_write, client_write, srv_read, client_read);
	ros::WallDuration(2.0).sleep();*/

	ros::waitForShutdown();
	return 0;
}
