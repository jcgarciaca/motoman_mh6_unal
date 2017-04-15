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
#include <fstream>

void moveRobot(move_group_interface::MoveGroup &group, std::vector<double> &group_variables_values, std::vector<double> nData);
void printCurrentJointPosition(move_group_interface::MoveGroup & group, std::vector<double> nData);
void printArray(std::vector<double> nData);

int main(int argc, char * argv[]){
 
/*
 std::cout << "Running random path node..." << std::endl;
 std::cout << "argc: " << argc << std::endl;
 for(int i = 0; i < argc; i++){
   std::cout << "argv[" << i << "]: " << argv[i] << std::endl;
 }
*/


 if(argc != 9){
   std::cout << "Error en el número de argumentos... " << argc << std::endl;
   return -1;
 }

 std::vector<double> nData;

 // change argv data to double
 for(int i = 1; i < argc; i++){
   nData.push_back(atof(argv[i]));
 }

// printArray(nData);


 ros::init(argc, argv, "test");
 ros::NodeHandle nh;
 ros::AsyncSpinner spinner(1);
 spinner.start();

 std::cout << "Running random path node..." << std::endl;
 move_group_interface::MoveGroup group("mh6");
 std::vector<double> group_variable_values;

 group.getCurrentState() -> copyJointGroupPositions(group.getCurrentState() -> getRobotModel() -> getJointModelGroup(group.getName()), group_variable_values);
 group.setPlannerId("RRTConnectkConfigDefault");

 std::cout << "----------------------------------------" << std::endl;
 std::cout << "Joint Tolerance: " << group.getGoalJointTolerance() << std::endl;
 std::cout << "Orientation Tolerance: " << group.getGoalOrientationTolerance() << std::endl;
 std::cout << "Position Tolerance: " << group.getGoalPositionTolerance() << std::endl;
 std::cout << "----------------------------------------" << std::endl;

 moveRobot(group, group_variable_values, nData);
 printCurrentJointPosition(group, nData);
 ros::waitForShutdown();

 return 0;
}


void moveRobot(move_group_interface::MoveGroup &group, std::vector<double> &group_variables_values, std::vector<double> nData){
 for(int i = 0; i < nData.size(); i++){
   group_variables_values[i] = nData[i];
 }
 group.setJointValueTarget(group_variables_values);
 moveit::planning_interface::MoveGroup::Plan my_plan;
 bool success = group.plan(my_plan);

 if(success){
   std::cout << "Planning successfully" << std::endl;
   group.move();
 }else
   std::cout << "Planning failed" << std::endl;
}

void printCurrentJointPosition(move_group_interface::MoveGroup & group, std::vector<double> nData){
 // open txt file
 std::ofstream myfile("positions.txt");

 std::vector<double> group_variable_values;
 group.getCurrentState() -> copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
 std::cout << "----------------------------------------" << std::endl;
 for(int i = 0; i < group_variable_values.size(); i++){
   std::cout << "joint[" << i << "]: " << group_variable_values[i] << std::endl;
   myfile << "joint[" << i << "]:\t" << group_variable_values[i] << "\t" << "comanded[" << i << "]:\t" << nData[i] << std::endl;
 }
 std::cout << "----------------------------------------" << std::endl;
 myfile.close();

}


void printArray(std::vector<double> nData){
 std::cout << "size of data" << nData.size() << std::endl;
 for(int i = 0; i < nData.size(); i++){
   std::cout << "array[" << i << "]: " << nData[i] << std::endl;
 }
}
