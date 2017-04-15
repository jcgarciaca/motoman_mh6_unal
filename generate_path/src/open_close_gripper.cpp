#include "ros/ros.h"
#include "motoman_msgs/WriteSingleIO.h"
#include "motoman_msgs/ReadSingleIO.h"
#include <cstdlib>
#include <iostream>


void open_gripper(motoman_msgs::WriteSingleIO & srv_write, ros::ServiceClient & client_write,
		motoman_msgs::ReadSingleIO & srv_read, ros::ServiceClient & client_read){
	srv_read.request.address = 20050;
	if(client_read.call(srv_read)){
		if(srv_read.response.value == 1){
			srv_write.request.address = 10032;
			srv_write.request.value = 0;
			if(client_write.call(srv_write)){
				std::cout << "Reset 10032" << std::endl;
			}else{
				ROS_ERROR("Failed to call service write_single_io");
			}
			srv_write.request.address = 10033;
			srv_write.request.value = 1;
			if(client_write.call(srv_write)){
				std::cout << "Set 10033" << std::endl;
			}else{
				ROS_ERROR("Failed to call service write_single_io");
			}
		}else{
			std::cout << "Gripper already open..." << std::endl;
		}
	}else{
		ROS_ERROR("Failed to call service read_single_io");
	}
}

void close_gripper(motoman_msgs::WriteSingleIO & srv_write, ros::ServiceClient & client_write,
		motoman_msgs::ReadSingleIO & srv_read, ros::ServiceClient & client_read){
	srv_read.request.address = 20050;
	if(client_read.call(srv_read)){
		if(srv_read.response.value == 0){
			srv_write.request.address = 10033;
			srv_write.request.value = 0;
			if(client_write.call(srv_write)){
				std::cout << "Reset 10033" << std::endl;
			}else{
				ROS_ERROR("Failed to call service write_single_io");
			}
			srv_write.request.address = 10032;
			srv_write.request.value = 1;
			if(client_write.call(srv_write)){
				std::cout << "Set 10032" << std::endl;
			}else{
				ROS_ERROR("Failed to call service write_single_io");
			}
		}else{
			std::cout << "Gripper already closed..." << std::endl;
		}
	}else{
		ROS_ERROR("Failed to call service read_single_io");
	}
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "open_close_gripper");
	ros::NodeHandle nh;
	motoman_msgs::WriteSingleIO srv_write;
	ros::ServiceClient client_write = nh.serviceClient<motoman_msgs::WriteSingleIO>("write_single_io");

	ros::ServiceClient client_read = nh.serviceClient<motoman_msgs::ReadSingleIO>("read_single_io");
	motoman_msgs::ReadSingleIO srv_read;

	open_gripper(srv_write, client_write, srv_read, client_read);
	ros::WallDuration(2.0).sleep();
	close_gripper(srv_write, client_write, srv_read, client_read);
	ros::WallDuration(2.0).sleep();

	ros::waitForShutdown();
	return 0;
}
