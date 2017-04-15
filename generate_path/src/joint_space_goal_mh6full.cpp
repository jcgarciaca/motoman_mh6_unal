#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char *argv[]){
 ros::init(argc, argv, "joint_space_goal_mh6full");
 ros::AsyncSpinner spinner(1);
 spinner.start();

 move_group_interface::MoveGroup group("arm_on_rail");

 std::vector<double> group_variable_values;

 group.getCurrentState() -> copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);


 geometry_msgs::PoseStamped pose_msg;
 pose_msg = group.getCurrentPose();

 std::cout << "Position X: " << pose_msg.pose.position.x << std::endl;
 std::cout << "Position Y: " << pose_msg.pose.position.y << std::endl;
 std::cout << "Position Z: " << pose_msg.pose.position.z << std::endl;

 std::cout << "Orientation X: " << pose_msg.pose.orientation.x << std::endl;
 std::cout << "Orientation Y: " << pose_msg.pose.orientation.y << std::endl;
 std::cout << "Orientation Z: " << pose_msg.pose.orientation.z << std::endl;
 std::cout << "Orientation W: " << pose_msg.pose.orientation.w << std::endl;


 for(int i = 0; i < group_variable_values.size(); i++){
   std::cout << "Joint[" << i << "]: " << group_variable_values[i] << std::endl;
 }

 group_variable_values[0] = -1.0;
 group_variable_values[1] = 1.0;
 group.setJointValueTarget(group_variable_values);

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
