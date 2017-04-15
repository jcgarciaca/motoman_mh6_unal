#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

const double PI = 3.14159265358979323846;

double rad2deg(double rad){
 double deg = rad * 180 / PI;
 return deg;
}

int main(int argc, char *argv[]){
 ros::init(argc, argv, "print_mh6full_positions");
 ros::AsyncSpinner spinner(1);
 spinner.start();
 
 move_group_interface::MoveGroup group("mh6");
 std::vector<double> group_variable_values;
 group.getCurrentState() -> copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

 for(int i = 0; i < group_variable_values.size(); i++){
   if(i == 0){
      std::cout << "Joint[" << i << "]: " << group_variable_values[i] << std::endl;
   }else{
      std::cout << "Joint[" << i << "]: " << rad2deg(group_variable_values[i]) << std::endl;
   }
 }

 group_variable_values[0] = -1.0;
 group_variable_values[1] = 0.5;
 group_variable_values[7] = 2.0;
 group.setJointValueTarget(group_variable_values);

 moveit::planning_interface::MoveGroup::Plan my_plan;
 bool success = group.plan(my_plan);

 if(success){
   ROS_INFO("Planning successfully");
   group.move();
 }else{
   ROS_INFO("Planning failed");
 }

 ros::waitForShutdown();
}
