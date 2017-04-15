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

void printPose(geometry_msgs::Pose &pose_msg);

int main(int argc, char *argv[]){
	ros::init(argc, argv, "jacobian");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	move_group_interface::MoveGroup group("arm_on_rail");
	group.setPlannerId("RRTConnectkConfigDefault");
	group.setPlanningTime(5.0);

	move_group_interface::MoveGroup group_move("mh6");
	group_move.setPlannerId("RRTConnectkConfigDefault");
	group_move.setPlanningTime(5.0);

	std::vector<double> group_variable_values, group_variable_values_full;

	// get current joint values
	group.getCurrentState() -> copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()),
			group_variable_values);
	group_move.getCurrentState() -> copyJointGroupPositions(group_move.getCurrentState()->getRobotModel()->getJointModelGroup(group_move.getName()),
			group_variable_values_full);

	std::cout << "size full: " << group_variable_values_full.size() << std::endl;

	// Get Current Pose
	geometry_msgs::PoseStamped pose_stmp_msg, start_pose, end_pose, tmp_pose;
	pose_stmp_msg = group.getCurrentPose("arm_link_tool0");
	geometry_msgs::Pose pose_msg = pose_stmp_msg.pose;

	printPose(pose_msg);

	// start pose
	start_pose = pose_stmp_msg;
	start_pose.pose.position.x = 2.805;
	start_pose.pose.position.y = 0.006;
	start_pose.pose.position.z = 1.595;
	start_pose.pose.orientation.x = 0;
	start_pose.pose.orientation.y = -1;
	start_pose.pose.orientation.z = 0;
	start_pose.pose.orientation.w = 0.00075;

	// tmp pose
	tmp_pose = pose_stmp_msg;
	tmp_pose.pose.position.x = 2.805;
	tmp_pose.pose.position.y = 0.691;
	tmp_pose.pose.position.z = 1.595;
	tmp_pose.pose.orientation.x = 0.0003;
	tmp_pose.pose.orientation.y = -0.9545;
	tmp_pose.pose.orientation.z = 0.2981;
	tmp_pose.pose.orientation.w = 0.00075;

	// end pose
	end_pose = pose_stmp_msg;
	end_pose.pose.position.x = 2.343;
	end_pose.pose.position.y = 0.370;
	end_pose.pose.position.z = 1.595;
	end_pose.pose.orientation.x = 0;
	end_pose.pose.orientation.y = -1;
	end_pose.pose.orientation.z = 0.0001;
	end_pose.pose.orientation.w = 0.0006;

	// IK
	// create server and client
	moveit_msgs::GetPositionIK server;
	ros::ServiceClient client = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

	///////////////////////////////////////////
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	robot_state::RobotStatePtr kinematic_state;//(new robot_state::RobotState(kinematic_model));
	//kinematic_state->setToDefaultValues();
	robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm_on_rail");

	kinematic_state = group.getCurrentState();

	std::vector<std::string> joint_names = joint_model_group->getJointModelNames();
	std::cout << "joint names: " << joint_names.size() << std::endl;

	for(int i = 0; i < joint_names.size(); i++){
		std::cout << joint_names[i] << std::endl;
	}

	Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
	Eigen::MatrixXd jacobian, jacobian_transpose, jacobian_aux, pseudo_inverse, delta_q, delta_X(6, 1);
/*
	kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
	                             reference_point_position,
	                             jacobian);
	ROS_INFO_STREAM("Jacobian: " << jacobian);
*/
	jacobian = kinematic_state->getJacobian(joint_model_group, reference_point_position);

	jacobian_transpose = jacobian.transpose();

	ROS_INFO_STREAM("jacobian_transpose: " << jacobian_transpose);

	jacobian_aux = jacobian.operator *(jacobian_transpose);

	ROS_INFO_STREAM("jacobian_aux: " << jacobian_aux);

	jacobian_aux = jacobian_aux.inverse();

	ROS_INFO_STREAM("jacobian_aux: " << jacobian_aux);

	pseudo_inverse = jacobian_transpose.operator *(jacobian_aux);

	ROS_INFO_STREAM("Jacobian: " << jacobian);
	ROS_INFO_STREAM("Pseudoinverse: " << pseudo_inverse);


	delta_X(0, 0) = 0.005;
	delta_X(1, 0) = 0.0;
	delta_X(2, 0) = 0.0;
	delta_X(3, 0) = 0.0;
	delta_X(4, 0) = 0.0;
	delta_X(5, 0) = 0.0;


	delta_q = pseudo_inverse.operator *(delta_X);
	ROS_INFO_STREAM("delta_q: " << delta_q);

	ROS_INFO_STREAM("deltaX: " << jacobian.operator *(delta_q));


	for(int i = 0; i < group_variable_values.size(); i++){
		group_variable_values_full[i] += delta_q(i, 0);
	}

	group_move.setJointValueTarget(group_variable_values_full);

	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success = group_move.plan(my_plan);

	if(success){
	   std::cout << "Planning successfully" << std::endl;
	   group_move.move();
	}else{
	   std::cout << "Planning failed" << std::endl;
	}

	ros::WallDuration(2.0).sleep();
	std::cout << "Complete!!" << std::endl;

	pose_stmp_msg = group.getCurrentPose("arm_link_tool0");
	pose_msg = pose_stmp_msg.pose;

	printPose(pose_msg);


	//////////////////////////////////////////

/*
	sensor_msgs::JointState joint_state;

	std::vector<std::vector<double> > joints_trajectory, joints_trajectory_tmp;

	calculateIK(server, client, group, pose_stmp_msg, joint_state, group_variable_values_full);

	calculateIK(server, client, group, start_pose, joint_state, group_variable_values_full);
	joints_trajectory_tmp.push_back(group_variable_values_full);

	calculateIK(server, client, group, tmp_pose, joint_state, group_variable_values_full);
	joints_trajectory_tmp.push_back(group_variable_values_full);

	calculateIK(server, client, group, end_pose, joint_state, group_variable_values_full);
	joints_trajectory_tmp.push_back(group_variable_values_full);


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


	// print trajectory
	std::cout << "size: " << joints_trajectory.size() << std::endl;

	for(int i = 0; i < joints_trajectory.size(); i++){
		std::cout << "punto: " << i << std::endl;
		for(int j = 0; j < group_variable_values_full.size(); j++){
			group_variable_values_full[j] = joints_trajectory[i][j];
			std::cout << group_variable_values_full[j] << std::endl;
		}
		std::cout << "--------------------------------------" << std::endl;
	}

	// save trajectory in joints
	std::ofstream myfile2 ("/home/juliocesar/joint_values.txt");

	if(myfile2.is_open()){
	    std::cout << "saving file" << std::endl;
	    for(int i = 0; i < joints_trajectory.size(); i++){
	      for(int j = 0; j < group_variable_values_full.size(); j++){
			group_variable_values_full[j] = joints_trajectory[i][j];
			myfile2 << group_variable_values_full[j] << "\t";
	      }
	      myfile2 << std::endl;
	    }
	    myfile2.close();
	}else{
	     std::cout << "error with file" << std::endl;
	}

	ROS_INFO("Start moving!!");


	// move robot for joints trajectory
	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success;
	for(int i = 0; i < joints_trajectory.size(); i++){
		//std::cout << "Punto " << i << std::endl;
		for(int j = 0; j < group_variable_values_full.size(); j++){
			group_variable_values_full[j] = joints_trajectory[i][j];
			//std::cout << group_variable_values[j] << std::endl;
		}
		group_move.setJointValueTarget(group_variable_values_full);
		success = group_move.plan(my_plan);
		if(success){
			ROS_INFO("Planning successfully");
			group_move.move();
			ros::WallDuration(2.0).sleep();
		}else{
			ROS_INFO("Planning failed");
		}
	}
*/
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
