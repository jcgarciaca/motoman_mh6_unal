#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char *argv[]){
 ros::init(argc, argv, "custom_path");
 ros::NodeHandle nh;
 ros::AsyncSpinner spinner(1);
 spinner.start();

 moveit::planning_interface::MoveGroup group("manipulator");
 moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

 ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
 moveit_msgs::DisplayTrajectory display_trajectory;

 geometry_msgs::Pose target_pose1;
 target_pose1.orientation.w = 0.726282;
 target_pose1.orientation.x = 4.04423e-07;
 target_pose1.orientation.y = -0.687396;
 target_pose1.orientation.z = 4.81813e-07;
 target_pose1.position.x = 0.0261186;
 target_pose1.position.y = 4.50972e-07;
 target_pose1.position.z = 0.573659;

 group.setPoseTarget(target_pose1);

 moveit::planning_interface::MoveGroup::Plan my_plan;
 bool success = group.plan(my_plan);
 ROS_INFO("Visualizing plan 1 (pose goal) %s", success?"":"FAILED");

 sleep(5.0);
 ros::shutdown();

 return 0;
}

