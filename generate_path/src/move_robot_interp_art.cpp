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

#include <tf/transform_datatypes.h>

#include <cstdlib>
#include <iostream>
#include <fstream>


int calculateIK(moveit_msgs::GetPositionIK &server, ros::ServiceClient &client,
		move_group_interface::MoveGroup &group, geometry_msgs::PoseStamped pose_stmp_msg,
		sensor_msgs::JointState &joint_state, std::vector<double> &group_variable_values);

int main(int argc, char *argv[]){
	ros::init(argc, argv, "move_robot_interp_art");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	move_group_interface::MoveGroup group("arm_on_rail");
	group.setPlannerId("RRTConnectkConfigDefault");
	group.setPlanningTime(5.0);

	std::vector<double> group_variable_values;
	
	// get current joint values
	group.getCurrentState() -> copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()),
			group_variable_values);

	// Get Current Pose
	geometry_msgs::PoseStamped pose_stmp_msg, start_pose, end_pose;
	pose_stmp_msg = group.getCurrentPose("arm_link_tool0");
	geometry_msgs::Pose pose_msg = pose_stmp_msg.pose;

	// start pose
	start_pose = pose_stmp_msg;
	start_pose.pose.position.x = 2.805;
	start_pose.pose.position.y = 0.006;
	start_pose.pose.position.z = 1.595;
	start_pose.pose.orientation.x = 0;
	start_pose.pose.orientation.y = -1;
	start_pose.pose.orientation.z = 0;
	start_pose.pose.orientation.w = 0.00075;

	// end pose
	end_pose = pose_stmp_msg;
	end_pose.pose.position.x = 2.805;
	end_pose.pose.position.y = 0.691;
	end_pose.pose.position.z = 1.595;
	end_pose.pose.orientation.x = 0.0003;
	end_pose.pose.orientation.y = -0.9545;
	end_pose.pose.orientation.z = 0.2981;
	end_pose.pose.orientation.w = 0.00075;

	// IK
	// create server and client
	moveit_msgs::GetPositionIK server;
	ros::ServiceClient client = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

	sensor_msgs::JointState joint_state;

	std::vector<std::vector<double> > joints_trajectory, joints_trajectory_tmp;

	calculateIK(server, client, group, pose_stmp_msg, joint_state, group_variable_values);

	calculateIK(server, client, group, start_pose, joint_state, group_variable_values);
	joints_trajectory_tmp.push_back(group_variable_values);

	calculateIK(server, client, group, end_pose, joint_state, group_variable_values);
	joints_trajectory_tmp.push_back(group_variable_values);


	// generate trajectory division
	joints_trajectory.push_back(joints_trajectory_tmp[0]);
	double interval = 3;
	std::vector<double> current_joints, next_joints;
	for(int i = 0; i < joints_trajectory_tmp.size() - 1; i++){
		current_joints = joints_trajectory_tmp[i];
		next_joints = joints_trajectory_tmp[i+1];
		double delta_values[current_joints.size()];
		for(int j = 0; j < current_joints.size(); j++){
			delta_values[j] = (next_joints[j] - current_joints[j]) / interval;
		}

		for(int index = 0; index < interval; index++){
			for(int j = 0; j < current_joints.size(); j++){
				current_joints[j] += delta_values[j];
			}
			joints_trajectory.push_back(current_joints);
		}
	}

/*
	// print trajectory
	std::cout << "size: " << joints_trajectory.size() << std::endl;

	for(int i = 0; i < joints_trajectory.size(); i++){
		std::cout << "punto: " << i << std::endl;
		for(int j = 0; j < group_variable_values.size(); j++){
			group_variable_values[j] = joints_trajectory[i][j];
			std::cout << group_variable_values[j] << std::endl;
		}
		std::cout << "--------------------------------------" << std::endl;
	}
*/

	// move robot for joints trajectory
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success;
	for(int i = 0; i < joints_trajectory.size(); i++){
		//std::cout << "Punto " << i << std::endl;
		for(int j = 0; j < group_variable_values.size(); j++){
			group_variable_values[j] = joints_trajectory[i][j];
			//std::cout << group_variable_values[j] << std::endl;
		}
		group.setJointValueTarget(group_variable_values);
		success = group.plan(my_plan);
		if(success){
			ROS_INFO("Planning successfully");
			group.move();
			//ros::WallDuration(2.0).sleep();
		}else{
			ROS_INFO("Planning failed");
		}
	}

	ros::waitForShutdown();
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

	 std::cout << "--------------------------------------" << std::endl;
	 if(server.response.error_code.val == 1){
		std::cout << " IK found !!" << std::endl;
		joint_state = server.response.solution.joint_state;
		for(int i = 0; i < group_variable_values.size(); i++){
			group_variable_values[i] = joint_state.position[i];
		}
		/*for(int i = 0; i < server.response.solution.joint_state.position.size(); i++){
			std::cout << "i: " << joint_state.position[i] << std::endl;
		}*/
		return 1;
	 }else{
		 std::cout << " No IK found" << std::endl;
		 return -1;
	 }
}
