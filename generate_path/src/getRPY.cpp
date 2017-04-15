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

void printPose(geometry_msgs::Pose &pose_msg);
void printAngle(double roll, double pitch, double yaw);

int main(int argc, char *argv[]){
	ros::init(argc, argv, "print_pose");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	move_group_interface::MoveGroup group("arm_on_rail");
	group.setPlannerId("RRTConnectkConfigDefault");

	// get current pose
	geometry_msgs::PoseStamped pose_stmp_msg;
	pose_stmp_msg = group.getCurrentPose("arm_link_tool0");
	geometry_msgs::Pose pose_msg = pose_stmp_msg.pose;
	

	printPose(pose_msg);

	double roll, pitch, yaw;
	tf::Quaternion q(pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z,
			pose_msg.orientation.w);

	tf::Matrix3x3 matrix(q);

//	matrix.getRPY(roll, pitch, yaw, 1);
	matrix.getRPY(roll, pitch, yaw);

	printAngle(roll, pitch, yaw);

//	matrix.getRPY(roll, pitch, yaw, 2);

//	printAngle(roll, pitch, yaw);

	ros::waitForShutdown();
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

void printAngle(double roll, double pitch, double yaw){
	std::cout << "--------------------------------------" << std::endl;
	std::cout << "---------------- Angle ----------------" << std::endl;
	std::cout << "--------------------------------------" << std::endl;
	std::cout << "roll: " << roll << std::endl;
	std::cout << "pitch: " << pitch << std::endl;
	std::cout << "yaw: " << yaw << std::endl;
	std::cout << "--------------------------------------" << std::endl;
}
