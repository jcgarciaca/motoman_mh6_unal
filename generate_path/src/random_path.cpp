#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

int main(int argc, char *argv[]){
 ros::init(argc, argv, "random_path");
 ros::AsyncSpinner spinner(1);
 spinner.start();
 
 move_group_interface::MoveGroup group("manipulator");
 group.setRandomTarget();
 
 group.move();
 ros::waitForShutdown();
}
