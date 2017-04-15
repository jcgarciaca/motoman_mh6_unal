#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char *argv[]){
 ros::init(argc, argv, "pose_goal_mh6full");
 ros::NodeHandle nh;
 ros::AsyncSpinner spinner(1);
 spinner.start();

 moveit::planning_interface::MoveGroup group("arm_on_rail");
 moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

 ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
 moveit_msgs::DisplayTrajectory display_trajectory;

 geometry_msgs::PoseStamped pose_msg;
 pose_msg = group.getCurrentPose();

 std::cout << "Position X: " << pose_msg.pose.position.x << std::endl;
 std::cout << "Position Y: " << pose_msg.pose.position.y << std::endl;
 std::cout << "Position Z: " << pose_msg.pose.position.z << std::endl;

 std::cout << "Orientation X: " << pose_msg.pose.orientation.x << std::endl;
 std::cout << "Orientation Y: " << pose_msg.pose.orientation.y << std::endl;
 std::cout << "Orientation Z: " << pose_msg.pose.orientation.z << std::endl;
 std::cout << "Orientation W: " << pose_msg.pose.orientation.w << std::endl;

 geometry_msgs::Pose target_pose;
 target_pose.position.x = 3.79;
 target_pose.position.y = 0.4;
 target_pose.position.z = 1.0;

 target_pose.orientation.x = 0;
 target_pose.orientation.y = 0;
 target_pose.orientation.z = -1;
 target_pose.orientation.w = 0;

 group.setPoseTarget(target_pose);

 moveit::planning_interface::MoveGroup::Plan my_plan;
 bool success = group.plan(my_plan);

 if(success){
   ROS_INFO("Planning successfully");
 }else{
   ROS_INFO("Planning failed");
 }

 group.move();
 ros::waitForShutdown();
}
