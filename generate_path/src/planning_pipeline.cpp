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

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

geometry_msgs::Pose getCurrentPose(moveit::planning_interface::MoveGroup &group, std::string end_effector){
	geometry_msgs::PoseStamped pose_msg;
	geometry_msgs::Pose group_pose;
	pose_msg = group.getCurrentPose(end_effector);

	group_pose = pose_msg.pose;
	return group_pose;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "planning_pipeline");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle node_handle("~");

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

	planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));

	ros::WallDuration sleep_time(2.0);
	sleep_time.sleep();

	planning_interface::MotionPlanRequest req;
	planning_interface::MotionPlanResponse res;

	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = "base_rail_link";
	pose.pose.position.x = 3.79;
	pose.pose.position.y = 0.4;
	pose.pose.position.z = 1.0;
	pose.pose.orientation.x = 0.0;
	pose.pose.orientation.y = 0.0;
	pose.pose.orientation.z = -1.0;
	pose.pose.orientation.w = 0.0;

	std::vector<double> tolerance_pose(3, 0.01);
	std::vector<double> tolerance_angle(3, 0.01);

	geometry_msgs::Quaternion qt = tf::createQuaternionMsgFromRollPitchYaw(0.4363, 0.0, 0.0);

	req.group_name = "mh6";
	moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("arm_link_tool0", pose, tolerance_pose, tolerance_angle);
	req.goal_constraints.push_back(pose_goal);

	pose_goal = kinematic_constraints::constructGoalConstraints("station_link_s1", pose, tolerance_pose, tolerance_angle);
	req.goal_constraints.push_back(pose_goal);

	planning_pipeline->generatePlan(planning_scene, req, res);

	if(res.error_code_.val != res.error_code_.SUCCESS){
		ROS_ERROR("Could not compute plan successfully");
		return 0;
	}

	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	ROS_INFO("Visualizing the trajectory");
	moveit_msgs::MotionPlanResponse response;
	res.getMessage(response);

	display_trajectory.trajectory_start = response.trajectory_start;
	display_trajectory.trajectory.push_back(response.trajectory);
	display_publisher.publish(display_trajectory);

	sleep_time.sleep();


	ros::waitForShutdown();
	return 0;
}
