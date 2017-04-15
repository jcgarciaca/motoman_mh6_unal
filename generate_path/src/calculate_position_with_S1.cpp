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

void printPose(geometry_msgs::Pose &pose_msg);
void transformPose(const tf::TransformListener& listener, geometry_msgs::PoseStamped &input_pose,
		geometry_msgs::PoseStamped &output_pose);

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

	std::cout << "header: " <<  pose_stmp_msg.header.frame_id << std::endl;
	transformPose(listener, pose_stmp_msg, output_pose);

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


	// init group move
	move_group_interface::MoveGroup group_move("mh6");
	group_move.setPlannerId("RRTConnectkConfigDefault");
	std::vector<double> group_variable_values;
	group.getCurrentState() -> copyJointGroupPositions(group.getCurrentState() -> getRobotModel() -> getJointModelGroup(group.getName()),
			group_variable_values);

	std::vector<double> s1_values;
	for(int i = 0; i < 34; i++){
		s1_values.push_back(0.02 * i - 0.34);
	}

	double s1 = 0.174533;
	double c_pos = cos(s1);
	double s_pos = sin(s1);

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

	std::cout << "Resultado: " << std::endl;
	printPose(modified_pose.pose);

/*
	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 4; j++){
			std::cout << T(i, j) << " ";
		}
		std::cout << std::endl;
	}
*/

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
		ROS_INFO("input_pose: (%.4f, %.4f. %.4f, %.4f, %.4f, %.4f, %.4f) -----> "
				"output_pose: (%.4f, %.4f. %.4f, %.4f, %.4f, %.4f, %.4f)", input_pose.pose.position.x,
				input_pose.pose.position.y, input_pose.pose.position.z, input_pose.pose.orientation.x,
				input_pose.pose.orientation.y, input_pose.pose.orientation.z, input_pose.pose.orientation.w,
				output_pose.pose.position.x, output_pose.pose.position.y, output_pose.pose.position.z,
				output_pose.pose.orientation.x, output_pose.pose.orientation.y,
				output_pose.pose.orientation.z, input_pose.pose.orientation.w);
	}catch(tf::TransformException &ex){
		ROS_ERROR("Received an exception: %s", ex.what());
	}
}
