#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

void transformPoint(const tf::TransformListener& listener){
	geometry_msgs::PointStamped output_point, input_point;
	input_point.header.frame_id = "arm_link_tool0";
	input_point.header.stamp = ros::Time();

	// point in base frame
	input_point.point.x = 0.0;
	input_point.point.y = 0.0;
	input_point.point.z = 0.0;

	try{
//		transformPoint (const std::string &target_frame, const geometry_msgs::PointStamped &stamped_in, geometry_msgs::PointStamped 					&stamped_out)
		listener.transformPoint("station_link_s1", input_point, output_point);
		ROS_INFO("input_point: (%.2f, %.2f. %.2f) -----> output_point: (%.4f, %.4f, %.4f) at time %.2f", input_point.point.x, input_point.point.y, input_point.point.z, output_point.point.x, output_point.point.y, output_point.point.z, output_point.header.stamp.toSec());
	}catch(tf::TransformException &ex){
		ROS_ERROR("Received an exception trying to transform a point from arm_link_tool0 to base_rail_link: %s", ex.what());
	}
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "tf_transform_listener");
	ros::NodeHandle nh;

	tf::TransformListener listener(ros::Duration(10));
	ros::Timer timer = nh.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

	ros::spin();

	return 0;
}

