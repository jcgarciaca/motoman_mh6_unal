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
	group_variable_values[0] = -1.5;
	group_variable_values[1] = -2.0615391731262207;
	group_variable_values[2] = 0.2781241238117218;
	group_variable_values[3] = -0.6519418358802795;
	group_variable_values[4] = 0.0;
	group_variable_values[5] = 0.9300635457038879;
	group_variable_values[6] = 0.4907577335834503;
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
	group_variable_values[0] = -1.0;
	group_variable_values[1] = -1.0820101499557495;
	group_variable_values[2] = 0.2588765025138855;
	group_variable_values[3] = -0.6773602962493896;
	group_variable_values[4] = 0.0;
	group_variable_values[5] = 0.9362702965736389;
	group_variable_values[6] = -0.48874810338020325;
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

void printCurrentJointPosition(move_group_interface::MoveGroup & group, std::string positionName){
	std::vector<double> group_variable_values;
	group.getCurrentState() -> copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
	std::cout << "Position " << positionName << std::endl;
	for(int i = 0; i < group_variable_values.size(); i++){
		std::cout << "joint[" << i << "]: " << group_variable_values[i] << std::endl;
	}
	std::cout << "---------------------------------------------------------" << std::endl;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "exercise_2");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	move_group_interface::MoveGroup group("mh6");
	std::vector<double> group_variable_values;
	group.getCurrentState() -> copyJointGroupPositions(group.getCurrentState() -> getRobotModel() -> getJointModelGroup(group.getName()), group_variable_values);

	/* available planners
	 * BKPIECEkConfigDefault
	 * ESTkConfigDefault
	 * KPIECEkConfigDefault
	 * LBKPIECEkConfigDefault
	 * PRMkConfigDefault
	 * PRMstarkConfigDefault
	 * RRTConnectkConfigDefault
	 * RRTkConfigDefault
	 * RRTstarkConfigDefault
	 * SBLkConfigDefault
	 * TRRTkConfigDefault
	 */

	std::cout << "argv[1]: " << argv[1] << std::endl;
	std::string planner_id;
	if(argc == 2){
		if(strcmp(argv[1], "1") == 0){
			std::cout << "planner # 1" << std::endl;
			planner_id = "BKPIECEkConfigDefault";
		}else if(strcmp(argv[1], "2") == 0){
			std::cout << "planner # 2" << std::endl;
			planner_id = "ESTkConfigDefault";
		}else if(strcmp(argv[1], "3") == 0){
			std::cout << "planner # 3" << std::endl;
			planner_id = "KPIECEkConfigDefault";
		}else if(strcmp(argv[1], "4") == 0){
			std::cout << "planner # 4" << std::endl;
			planner_id = "LBKPIECEkConfigDefault";
		}else if(strcmp(argv[1], "5") == 0){
			std::cout << "planner # 5" << std::endl;
			planner_id = "PRMkConfigDefault";
		}else if(strcmp(argv[1], "6") == 0){
			std::cout << "planner # 6" << std::endl;
			planner_id = "PRMstarkConfigDefault";
		}else if(strcmp(argv[1], "7") == 0){
			std::cout << "planner # 7" << std::endl;
			planner_id = "RRTConnectkConfigDefault";
		}else if(strcmp(argv[1], "8") == 0){
			std::cout << "planner # 8" << std::endl;
			planner_id = "RRTkConfigDefault";
		}else if(strcmp(argv[1], "9") == 0){
			std::cout << "planner # 9" << std::endl;
			planner_id = "RRTstarkConfigDefault";
		}else if(strcmp(argv[1], "10") == 0){
			std::cout << "planner # 10" << std::endl;
			planner_id = "SBLkConfigDefault";
		}else if(strcmp(argv[1], "11") == 0){
			std::cout << "planner # 11" << std::endl;
			planner_id = "TRRTkConfigDefault";
		}else{
			std::cout << "planner # 0" << std::endl;
			planner_id = "RRTConnectkConfigDefault";
		}
	}else{
		planner_id = "RRTConnectkConfigDefault";
	}

	std::cout << "planner id: " << planner_id << std::endl;

	group.setPlannerId(planner_id);
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

	// create objects scene
	ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
	ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);
	ros::WallDuration(1.0).sleep();

	moveit_msgs::CollisionObject co;
	co.header.stamp = ros::Time::now();
	co.header.frame_id = "base_rail_link";

	// remove pole
	co.id = "pole";
	co.operation = moveit_msgs::CollisionObject::REMOVE;
	pub_co.publish(co);

	// add pole
	co.operation = moveit_msgs::CollisionObject::ADD;
	co.primitives.resize(1);
	co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.1;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.3;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
	co.primitive_poses.resize(1);
	co.primitive_poses[0].position.x = 0.8;
	co.primitive_poses[0].position.y = -0.8;
	co.primitive_poses[0].position.z = 1.0;
	pub_co.publish(co);

	// remove table
	co.id = "table";
	co.operation = moveit_msgs::CollisionObject::REMOVE;
	pub_co.publish(co);

	// add table
	co.operation = moveit_msgs::CollisionObject::ADD;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1.5;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.5;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.9;
	co.primitive_poses[0].position.x = 0.8;
	co.primitive_poses[0].position.y = -0.8;
	co.primitive_poses[0].position.z = 0.45;
	pub_co.publish(co);

	// actions
	open_gripper(srv_write, client_write, srv_read, client_read);
	moveHome(group, group_variable_values);
	ros::WallDuration(1.0).sleep();
	printCurrentJointPosition(group, "home");
	//printPose(group);
	movePick(group, group_variable_values);
	ros::WallDuration(1.0).sleep();
	printCurrentJointPosition(group, "pick");
	//printPose(group);
	close_gripper(srv_write, client_write, srv_read, client_read);
	movePlace(group, group_variable_values);
	ros::WallDuration(1.0).sleep();
	printCurrentJointPosition(group, "place");
	//printPose(group);
	open_gripper(srv_write, client_write, srv_read, client_read);
	moveHome(group, group_variable_values);

	ros::waitForShutdown();
	return 0;
}
