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
void transformPose(const tf::TransformListener& listener, geometry_msgs::PoseStamped &input_pose,
		geometry_msgs::PoseStamped &output_pose);
int calculateIK(moveit_msgs::GetPositionIK &server, ros::ServiceClient &client,
		move_group_interface::MoveGroup &group, geometry_msgs::PoseStamped pose_stmp_msg,
		sensor_msgs::JointState &joint_state, std::vector<double> &group_variable_values);

void createTrackingPoses(geometry_msgs::Pose &current_pose,
		std::vector<geometry_msgs::Pose> & tracking_poses);
void printTrackingPoses(std::vector<geometry_msgs::Pose> & tracking_poses);

double calculateDistance(geometry_msgs::Pose &target_pose, geometry_msgs::Pose &current_pose);

int generatePseudoTargetPose(geometry_msgs::Pose &AGV_pose, geometry_msgs::PoseStamped &current_pose,
		geometry_msgs::PoseStamped &pseudo_target_pose);
void generateIncTargetPose(geometry_msgs::Pose &target_pose, geometry_msgs::PoseStamped &current_pose,
		geometry_msgs::PoseStamped &inc_target_pose, double &tolerance, double &desplazamiento);
void printTargetPose(geometry_msgs::Pose & target_pose);

int main(int argc, char *argv[]){
	ros::init(argc, argv, "tf_transform_listener");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	tf::TransformListener listener(ros::Duration(10));

	move_group_interface::MoveGroup group("arm_on_rail");
	group.setPlannerId("RRTConnectkConfigDefault");

	// get current pose
	geometry_msgs::PoseStamped pose_stmp_msg, pose_stmp_msg_aux, inc_target_pose;
	pose_stmp_msg = group.getCurrentPose("arm_link_tool0");
	geometry_msgs::Pose pose_msg = pose_stmp_msg.pose;

	sensor_msgs::JointState joint_state;

	printPose(pose_msg);

	std::vector<geometry_msgs::Pose> tracking_poses;

	// generate tracking points
	createTrackingPoses(pose_msg, tracking_poses);

	// print tracking poses
	printTrackingPoses(tracking_poses);

	// init group move
	move_group_interface::MoveGroup group_move("mh6");
	group_move.setPlannerId("RRTConnectkConfigDefault");
	std::vector<double> group_variable_values_full;

	group_move.getCurrentState() -> copyJointGroupPositions(group_move.getCurrentState()->getRobotModel()->getJointModelGroup(group_move.getName()),
			group_variable_values_full);

	moveit_msgs::GetPositionIK server;
	ros::ServiceClient client = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

	double tolerance = 0.0005, deltaDesplazamiento = 0.001;

	moveit::planning_interface::MoveGroup::Plan my_plan;

/*
	if(calculateIK(server, client, group, target_pose, joint_state, group_variable_values_full)){
		group_variable_values_full[7] = 0.0;

		group_move.setJointValueTarget(group_variable_values_full);
		success = group_move.plan(my_plan);
	}
*/

	//double distanceToPoint;

	std::cout << "tamaño " << tracking_poses.size() << std::endl;
	//bool completed = false;
	//while(ros::ok() && !completed){
		for(int i = 0; i < tracking_poses.size(); i++){
			std::cout << "entra a FOR" << std::endl;

			while(calculateDistance(tracking_poses[i], pose_stmp_msg_aux.pose) > tolerance){
				pose_stmp_msg_aux = group.getCurrentPose("arm_link_tool0");
				generateIncTargetPose(tracking_poses[i], pose_stmp_msg_aux, inc_target_pose,
						tolerance, deltaDesplazamiento);

				//std::cout << "inc target ---------" << std::endl;
				//printTargetPose(inc_target_pose.pose);
				if(calculateIK(server, client, group, inc_target_pose, joint_state, group_variable_values_full)){
					group_variable_values_full[7] = 0.0;
					group_move.setJointValueTarget(group_variable_values_full);
					bool success = group_move.plan(my_plan);
					if(success){
					  //std::cout << "Planning successfully" << std::endl;
					  group_move.move();
					  //pose_stmp_msg_aux = group.getCurrentPose("arm_link_tool0");
					  //while(calculateDistance(inc_target_pose.pose, pose_stmp_msg_aux.pose) > tolerance){
					  //	std::cout << "inside inner while" << std::endl;
					  //}
					  //ros::WallDuration(0.1).sleep();
					  robot_state::RobotState inc_start_state(*group_move.getCurrentState());
					  group_move.setStartState(inc_start_state);
					  ros::WallDuration(0.3).sleep();
					}else{
						std::cout << "Planning failed" << std::endl;
					}

					//ros::WallDuration(0.5).sleep();
					//ros::WallDuration(1.0).sleep();
				}else{
					std::cout << "IK NOT FOUND -----------" << std::endl;
				}

			}
			std::cout << "Point " << i+1 << " reached ---------------" << std::endl;
		}
		//completed = true;
	//}

	std::cout << "--------------- Complete ---------------------" << std::endl;
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
	input_pose.header.stamp = ros::Time();
	try{
		listener.transformPose("station_link_s1", input_pose, output_pose);
	}catch(tf::TransformException &ex){
		ROS_ERROR("Received an exception: %s", ex.what());
	}
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


double calculateDistance(geometry_msgs::Pose &target_pose, geometry_msgs::Pose &current_pose){

	return sqrt(pow(target_pose.position.x - current_pose.position.x, 2) +
			pow(target_pose.position.y - current_pose.position.y, 2) +
			pow(target_pose.position.z - current_pose.position.z, 2));
}

int generatePseudoTargetPose(geometry_msgs::Pose &AGV_pose, geometry_msgs::PoseStamped &current_pose,
		geometry_msgs::PoseStamped &pseudo_target_pose){

	pseudo_target_pose = current_pose;

	double deltaX = AGV_pose.position.x - current_pose.pose.position.x;
	double deltaY = AGV_pose.position.y - current_pose.pose.position.y;
	double deltaZ = AGV_pose.position.z - current_pose.pose.position.z;

	pseudo_target_pose.pose.position.x += deltaX / 2;
	pseudo_target_pose.pose.position.y += deltaY / 2;
	pseudo_target_pose.pose.position.z += deltaZ / 2;

	double tolerance = 0.005;

	// 1 toma AGV
	if(calculateDistance(AGV_pose, pseudo_target_pose.pose) < tolerance){
		pseudo_target_pose.pose = AGV_pose;
	}

	return 1;
}

void createTrackingPoses(geometry_msgs::Pose &current_pose,
		std::vector<geometry_msgs::Pose> & tracking_poses){

	geometry_msgs::Pose aux_pose = current_pose;
	double deltaDistance = 0.1;// meters

	// pto 1
	aux_pose.position.x += (deltaDistance/2);
	aux_pose.position.y += (deltaDistance/2);
	tracking_poses.push_back(aux_pose);
	// pto 2
	aux_pose.position.y -= deltaDistance;
	tracking_poses.push_back(aux_pose);
	// pto 3
	aux_pose.position.x -= deltaDistance;
	tracking_poses.push_back(aux_pose);
	// pto 4
	aux_pose.position.y += deltaDistance;
	tracking_poses.push_back(aux_pose);
	// pto 5 = pto1
	aux_pose.position.x += deltaDistance;
	tracking_poses.push_back(aux_pose);
}

void printTrackingPoses(std::vector<geometry_msgs::Pose> & tracking_poses){
	for(int i = 0; i < tracking_poses.size(); i++){
		std::cout << "--------------------------------------" << std::endl;
		std::cout << "Punto " << i + 1 << " ----------" << std::endl;
		std::cout << "x: " << tracking_poses[i].position.x << std::endl;
		std::cout << "y: " << tracking_poses[i].position.y << std::endl;
		std::cout << "z: " << tracking_poses[i].position.z << std::endl;

		std::cout << "Rx: " << tracking_poses[i].orientation.x << std::endl;
		std::cout << "Ry: " << tracking_poses[i].orientation.y << std::endl;
		std::cout << "Rz: " << tracking_poses[i].orientation.z << std::endl;
		std::cout << "Rw: " << tracking_poses[i].orientation.w << std::endl;
		std::cout << "--------------------------------------" << std::endl;
	}
}

void printTargetPose(geometry_msgs::Pose & target_pose){

	std::cout << "--------------------------------------" << std::endl;
	std::cout << "Pose " << std::endl;
	std::cout << "x: " << target_pose.position.x << std::endl;
	std::cout << "y: " << target_pose.position.y << std::endl;
	std::cout << "z: " << target_pose.position.z << std::endl;

	std::cout << "Rx: " << target_pose.orientation.x << std::endl;
	std::cout << "Ry: " << target_pose.orientation.y << std::endl;
	std::cout << "Rz: " << target_pose.orientation.z << std::endl;
	std::cout << "Rw: " << target_pose.orientation.w << std::endl;
	std::cout << "--------------------------------------" << std::endl;

}

void generateIncTargetPose(geometry_msgs::Pose &target_pose, geometry_msgs::PoseStamped &current_pose,
		geometry_msgs::PoseStamped &inc_target_pose, double &tolerance, double &desplazamiento){

	inc_target_pose = current_pose;
	double distance = calculateDistance(target_pose, current_pose.pose);
	double distance_unit_mm = distance / desplazamiento;
/*
	std::cout << "target pose -----------" << std::endl;
	printTargetPose(target_pose);
	std::cout << "current pose -----------" << std::endl;
	printTargetPose(current_pose.pose);
*/

	double deltaX = (target_pose.position.x - current_pose.pose.position.x) / distance_unit_mm;
	double deltaY = (target_pose.position.y - current_pose.pose.position.y) / distance_unit_mm;
	double deltaZ = (target_pose.position.z - current_pose.pose.position.z) / distance_unit_mm;
/*
	std::cout << "distance: " << distance << std::endl;
	std::cout << "distance_unit_mm: " << distance_unit_mm << std::endl;
	std::cout << "deltaX: " << deltaX << std::endl;
	std::cout << "deltaY: " << deltaY << std::endl;
	std::cout << "deltaZ: " << deltaZ << std::endl;
*/
	if(distance > tolerance){
		inc_target_pose.pose.position.x += deltaX;
		inc_target_pose.pose.position.y += deltaY;
		inc_target_pose.pose.position.z += deltaZ;
	}else{
		inc_target_pose.pose = target_pose;
	}
}
