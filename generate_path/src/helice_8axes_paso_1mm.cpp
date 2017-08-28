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


void printPose(geometry_msgs::Pose &pose_msg);
void transformPose(const tf::TransformListener& listener, geometry_msgs::PoseStamped &input_pose,
		geometry_msgs::PoseStamped &output_pose);
void calculateModifiedPose(double s1, geometry_msgs::PoseStamped &output_pose,
		geometry_msgs::PoseStamped &modified_pose);
int calculateIK(moveit_msgs::GetPositionIK &server, ros::ServiceClient &client,
		move_group_interface::MoveGroup &group, geometry_msgs::PoseStamped pose_stmp_msg,
		sensor_msgs::JointState &joint_state, std::vector<double> &group_variable_values);
int generateHelicoidalPath(geometry_msgs::PoseStamped &center_pose,
		std::vector<geometry_msgs::PoseStamped> &output_pose);
int generateHelicoidalPath(geometry_msgs::PoseStamped &center_pose, std::vector<double> &s1,
		std::vector<geometry_msgs::PoseStamped> &output_pose);
int generateHelicoidalPathConvex(geometry_msgs::PoseStamped &center_pose, std::vector<double> &s1,
		std::vector<geometry_msgs::PoseStamped> &output_pose);
void compensateRotation(double s1, geometry_msgs::PoseStamped &pose_msg);

int main(int argc, char *argv[]){
	ros::init(argc, argv, "tf_transform_listener");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	tf::TransformListener listener(ros::Duration(10));

	move_group_interface::MoveGroup group("arm_on_rail");
	group.setPlannerId("RRTConnectkConfigDefault");

	// get current pose
	geometry_msgs::PoseStamped pose_stmp_msg, output_pose, modified_pose;
	pose_stmp_msg = group.getCurrentPose("arm_link_tool0");
	geometry_msgs::Pose pose_msg = pose_stmp_msg.pose;
	modified_pose = pose_stmp_msg;

	printPose(pose_msg);

	std::vector<geometry_msgs::PoseStamped> helicoidal_path_poses;
	std::vector<double> s1_values;

	generateHelicoidalPath(pose_stmp_msg, s1_values, helicoidal_path_poses);

	std::cout << "size of path: " << helicoidal_path_poses.size() << std::endl;
	std::cout << "size of angles: " << s1_values.size() << std::endl;


/*
	for(int i = 0; i < helicoidal_path_poses.size(); i++){
		std::cout << helicoidal_path_poses[i].pose.position.x << "\t" <<
				helicoidal_path_poses[i].pose.position.y << "\t" <<
				helicoidal_path_poses[i].pose.position.z << std::endl;
	}
*/



	// init group move
	move_group_interface::MoveGroup group_move("mh6");
	group_move.setPlannerId("RRTConnectkConfigDefault");
	std::vector<double> group_variable_values, group_variable_values_full, group_variable_values_prev;

	group.getCurrentState() -> copyJointGroupPositions(group.getCurrentState() -> getRobotModel() -> getJointModelGroup(group.getName()),
			group_variable_values);
	group_move.getCurrentState() -> copyJointGroupPositions(group_move.getCurrentState()->getRobotModel()->getJointModelGroup(group_move.getName()),
			group_variable_values_full);

	group_variable_values_prev = group_variable_values_full;



/*
	for(int i = 0; i < 34; i++){
		s1_values.push_back(0.02 * i - 0.34);
	}

	for(int i = 0; i < 34; i++){
		s1_values.push_back(- 0.02 * i + 0.34);
	}
*/

	moveit_msgs::GetPositionIK server;
	ros::ServiceClient client = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

	sensor_msgs::JointState joint_state;
	std::vector<std::vector<double> > joints_trajectory;


	for(int i = 0; i < helicoidal_path_poses.size(); i++){
		transformPose(listener, helicoidal_path_poses[i], output_pose);
		compensateRotation(s1_values[i], output_pose);
		calculateModifiedPose(s1_values[i], output_pose, modified_pose);
		if(calculateIK(server, client, group, modified_pose, joint_state, group_variable_values_full)){
			group_variable_values_full[7] = s1_values[i];
			joints_trajectory.push_back(group_variable_values_full);
		}
	}


	moveit::planning_interface::MoveGroup::Plan my_plan;
	bool success;
	moveit_msgs::RobotTrajectory trajectory;
	robot_trajectory::RobotTrajectory traj_full(group_move.getCurrentState()->getRobotModel(), "mh6");
	robot_trajectory::RobotTrajectory traj_aux(group_move.getCurrentState()->getRobotModel(), "mh6");

	robot_state::RobotState virtual_start_state(*group_move.getCurrentState());
	robot_state::RobotState real_start_state(*group_move.getCurrentState());

	for(int i = 0; i < joints_trajectory.size(); i++){
		//std::cout << i << ": " << " de " << joints_trajectory.size() << std::endl;

		if(i > 0){
			group_variable_values_prev = group_variable_values_full;
			virtual_start_state.setJointGroupPositions(group_move.getName(), group_variable_values_prev);
			group_move.setStartState(virtual_start_state);
		}


		for(int j = 0; j < group_variable_values_full.size(); j++){
			group_variable_values_full[j] = joints_trajectory[i][j];
		}

		group_move.setJointValueTarget(group_variable_values_full);
		success = group_move.plan(my_plan);
		if(success){

			trajectory = my_plan.trajectory_;
			traj_aux.setRobotTrajectoryMsg(*group_move.getCurrentState(), trajectory);
			traj_full.append(traj_aux, traj_aux.getWaypointDurationFromStart(traj_aux.getWayPointCount()));

		}else{
			ROS_INFO("Planning failed");
		}
	}

	ROS_INFO("Move");

	moveit_msgs::ExecuteKnownTrajectory srv;
	srv.request.wait_for_execution = true;
	ros::ServiceClient executeKnownTrajectoryServiceClient = nh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(
	      "/execute_kinematic_path");

	traj_full.getRobotTrajectoryMsg(trajectory);

	group_move.setStartState(real_start_state);

	srv.request.trajectory = trajectory;

	executeKnownTrajectoryServiceClient.call(srv);

	ROS_INFO("Trajectory Executed!!");

	pose_stmp_msg = group.getCurrentPose("arm_link_tool0");
	pose_msg = pose_stmp_msg.pose;
	printPose(pose_msg);


	ROS_INFO("Finish");
	ros::waitForShutdown();

	ros::spin();

	return 0;
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


void transformPose(const tf::TransformListener& listener, geometry_msgs::PoseStamped &input_pose,
		geometry_msgs::PoseStamped &output_pose){
//	geometry_msgs::PoseStamped output_pose;
	input_pose.header.stamp = ros::Time();
	try{
//		transformPose (const std::string &target_frame, const geometry_msgs::PoseStamped &stamped_in, geometry_msgs::PoseStamped &stamped_out)
		listener.transformPose("station_link_s1", input_pose, output_pose);
/*		ROS_INFO("input_pose: (%.4f, %.4f. %.4f, %.4f, %.4f, %.4f, %.4f) -----> "
				"output_pose: (%.4f, %.4f. %.4f, %.4f, %.4f, %.4f, %.4f)", input_pose.pose.position.x,
				input_pose.pose.position.y, input_pose.pose.position.z, input_pose.pose.orientation.x,
				input_pose.pose.orientation.y, input_pose.pose.orientation.z, input_pose.pose.orientation.w,
				output_pose.pose.position.x, output_pose.pose.position.y, output_pose.pose.position.z,
				output_pose.pose.orientation.x, output_pose.pose.orientation.y,
				output_pose.pose.orientation.z, input_pose.pose.orientation.w);
*/
	}catch(tf::TransformException &ex){
		ROS_ERROR("Received an exception: %s", ex.what());
	}
}

void calculateModifiedPose(double s1, geometry_msgs::PoseStamped &output_pose,
		geometry_msgs::PoseStamped &modified_pose){

	double c_pos = cos(s1);
	double s_pos = sin(s1);

	tf::Quaternion q(output_pose.pose.orientation.x, output_pose.pose.orientation.y,
			output_pose.pose.orientation.z, output_pose.pose.orientation.w);
	tf::Matrix3x3 R3_4(q);

	tf::Vector3 origin3_4(output_pose.pose.position.x, output_pose.pose.position.y, output_pose.pose.position.z);

	// create Transform Matrix
//	std::cout << "transformation matrix: " << std::endl;
	Eigen::MatrixXd T1_3 = Eigen::MatrixXd::Identity(4, 4);
	tf::Vector3 origin1_3(-0.7553, 0.9886, 1.0066);
	for(int i = 0; i < 3; i++){
		T1_3(i, 3) = origin1_3[i];
	}

	T1_3(1, 1) = c_pos; //cos\theta
	T1_3(1, 2) = -s_pos; //-sin\theta
	T1_3(2, 1) = s_pos; //sin\theta
	T1_3(2, 2) = c_pos; //cos\theta


	Eigen::MatrixXd T3_4 = Eigen::MatrixXd::Identity(4, 4);
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			T3_4(i, j) = R3_4[i][j];
		}
		T3_4(i, 3) = origin3_4[i];
	}

	Eigen::MatrixXd T1_4 = T1_3.operator *(T3_4);

	tf::Matrix3x3 R_final;
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			R_final[i][j] = T1_4(i, j);
		}
	}

	tf::Quaternion q_aux;
	R_final.getRotation(q_aux);
	geometry_msgs::Quaternion q_final;
	q_final.x = q_aux.getX();
	q_final.y = q_aux.getY();
	q_final.z = q_aux.getZ();
	q_final.w = q_aux.getW();
	modified_pose.pose.orientation = q_final;

	modified_pose.pose.position.x = T1_4(0, 3);
	modified_pose.pose.position.y = T1_4(1, 3);
	modified_pose.pose.position.z = T1_4(2, 3);

	//std::cout << "Resultado: " << std::endl;
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
		/*for(int i = 0; i < server.response.solution.joint_state.position.size(); i++){
			std::cout << "i: " << joint_state.position[i] << std::endl;
		}*/
		return 1;
	 }else{
		 std::cout << " ------- No IK found -------" << std::endl;
		 return -1;
	 }
}

int generateHelicoidalPath(geometry_msgs::PoseStamped &center_pose,
		std::vector<geometry_msgs::PoseStamped> &output_pose){

	// parameters
	double r = 0.01, k = 0.001, ind = 0.0;
	std::vector<double> t;//, x, y, z;

	double centerX, centerY, centerZ;

	geometry_msgs::PoseStamped current_pose = center_pose;

	// center is current position
	centerX = center_pose.pose.position.x;
	centerY = center_pose.pose.position.y;
	centerZ = center_pose.pose.position.z;

	// fill time
	while(ind < 20){
		t.push_back(ind);
		ind += 0.01;
	}

	for(int i = 0; i < t.size(); i++){
		double z_aux = r * cos(t[i]);
		double y_aux = r * sin(t[i]);
		double x_aux = k * t[i];

		current_pose.pose.position.x = centerX + x_aux;
		current_pose.pose.position.y = centerY + y_aux;
		current_pose.pose.position.z = centerZ + z_aux;

		output_pose.push_back(current_pose);
	}
	return 1;
}


int generateHelicoidalPath(geometry_msgs::PoseStamped &center_pose, std::vector<double> &s1,
		std::vector<geometry_msgs::PoseStamped> &output_pose){

	// parameters
//	double r = 0.01, k = 0.001, ind = 0.0;
	double r = 0.01, k = 0.0002, max = 88, ind = 0.0;
	std::vector<double> t;//, x, y, z;

	double centerX, centerY, centerZ;

	geometry_msgs::PoseStamped current_pose = center_pose;

	// center is current position
	centerX = center_pose.pose.position.x;
	centerY = center_pose.pose.position.y;
	centerZ = center_pose.pose.position.z;

	// fill time
	//double max = 20;
	double s1_value = 0.0, s1_value_max = 0.0, s1_value_min = 0.0;

	while(ind < max){
		t.push_back(ind);
		if(ind < max/4){
			s1_value += 0.00014;
		}else if(ind < (3*max/4)){
			s1_value -= 0.00014;
		}else{
			s1_value += 0.00014;
		}

		if(s1_value > s1_value_max){
			s1_value_max = s1_value;
		}

		if(s1_value < s1_value_min){
			s1_value_min = s1_value;
		}

		s1.push_back(s1_value);
		ind += 0.01;
	}

	std::cout << "max: " << s1_value_max << std::endl;
	std::cout << "min: " << s1_value_min << std::endl;

	for(int i = 0; i < t.size(); i++){
		double z_aux = r * cos(t[i]);
		double y_aux = r * sin(t[i]);
		double x_aux = k * t[i];

		current_pose.pose.position.x = centerX + x_aux;
		current_pose.pose.position.y = centerY + y_aux;
		current_pose.pose.position.z = centerZ + z_aux;

		output_pose.push_back(current_pose);
	}
	return 1;
}

void compensateRotation(double s1, geometry_msgs::PoseStamped &pose_msg){
	//1.3334 * s1.^2 -1.5678* s1 + 0.4018
	double errorX = 1.3334 * pow(s1, 2) - 1.5678 * s1 + 0.4018;

	//1.158 * s1.^2 - 7.7285 * s1 - 0.0096
	double errorY = 1.158 * pow(s1, 2) - 7.7285 * s1 - 0.0096;

	//1.4086 * s1.^4 + 3.0528 * s1.^3 -3.6942 * s1.^2 -3.2571 * s1 + 0.0195
	double errorZ = 1.4086 * pow(s1, 4) + 3.0528 * pow(s1, 3) - 3.6942 * pow(s1, 2) -3.2571 * s1 + 0.0195;

	errorX /= 1000;
	errorY /= 1000;
	errorZ /= 1000;

	pose_msg.pose.position.x -= errorX;
	pose_msg.pose.position.y -= errorY;
	pose_msg.pose.position.z -= errorZ;
}

int generateHelicoidalPathConvex(geometry_msgs::PoseStamped &center_pose, std::vector<double> &s1,
		std::vector<geometry_msgs::PoseStamped> &output_pose){


	// parameters
	double r = 0.01, k = 0.0002, max = 28, ind = 0.0;
	std::vector<double> u;//, x, y, z;
	int index = 0;

	double centerX, centerY, centerZ;

	geometry_msgs::PoseStamped current_pose = center_pose;

	// center is current position
	centerX = center_pose.pose.position.x;
	centerY = center_pose.pose.position.y;
	centerZ = center_pose.pose.position.z;

	double s1_value = 0.0, s1_value_max = 0.0, s1_value_min = 0.0;

	double du = 0.01, u_aux = 1.57;
	for (int i = 0; i < 314; i++){
		u.push_back(u_aux);
		u_aux += du;
	}

	double dv = 0.0;

	for(int v = 0; v < max; v++){
		for(int i = 0; i < u.size(); i++){
			double z_aux = (-r * cos(u[i])) - r;
			double y_aux = 0.0;
			if((v%2) == 0){
				y_aux = (r * sin(u[i])) + r;
			}else{
				y_aux = (-r * sin(u[i])) + r;
			}

			double x_aux = dv;

			current_pose.pose.position.x = centerX + x_aux;
			current_pose.pose.position.y = centerY + y_aux;
			current_pose.pose.position.z = centerZ + z_aux;
			output_pose.push_back(current_pose);

			if(ind < max/4){
				s1_value += 0.00014;
			}else if(ind < (3*max/4)){
				s1_value -= 0.00014;
			}else{
				s1_value += 0.00014;
			}

			if(s1_value > s1_value_max){
				s1_value_max = s1_value;
			}

			if(s1_value < s1_value_min){
				s1_value_min = s1_value;
			}

			s1.push_back(s1_value);

			dv += 0.000002;
		}
		ind += 1;
	}

	std::cout << "max: " << s1_value_max << std::endl;
	std::cout << "min: " << s1_value_min << std::endl;

	return 1;
}
