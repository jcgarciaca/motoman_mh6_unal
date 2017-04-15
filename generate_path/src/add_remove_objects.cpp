#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char *argv[]){
 ros::init(argc, argv, "add_remove_objects");
 ros::AsyncSpinner spinner(1);
 spinner.start();

 move_group_interface::MoveGroup group("manipulator");
 moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

 sleep(5.0);

 moveit_msgs::CollisionObject collision_object;
 collision_object.id = "box1";
 collision_object.header.frame_id = group.getPlanningFrame();

 std::cout << "frame: " << group.getPlanningFrame() << std::endl;

 
 shape_msgs::SolidPrimitive primitive;
 primitive.type = primitive.BOX;
 primitive.dimensions.resize(3);
 primitive.dimensions[0] = 0.4;
 primitive.dimensions[1] = 0.1;
 primitive.dimensions[2] = 0.4;

 geometry_msgs::Pose box_pose;
 box_pose.orientation.w = 1.0;
 box_pose.position.x = 0.6;
 box_pose.position.y = -0.4;
 box_pose.position.z = 0.4;

 collision_object.primitives.push_back(primitive);
 collision_object.primitive_poses.push_back(box_pose);
 collision_object.operation = collision_object.ADD;

 std::vector<moveit_msgs::CollisionObject> collision_objects;
 collision_objects.push_back(collision_object);

 ROS_INFO("Add an object into the world");
 planning_scene_interface.addCollisionObjects(collision_objects);

 sleep(2.0);

/*
 group.setPlanningTime(10.0);

 geometry_msgs::Pose target_pose1;
 target_pose1.orientation.w = 1.0;
 target_pose1.position.x = 0.28;
 target_pose1.position.y = -0.7;
 target_pose1.position.z = 0.2;

 group.setStartState(*group.getCurrentState());
 group.setPoseTarget(target_pose1);

 moveit::planning_interface::MoveGroup::Plan my_plan;
 bool success = group.plan(my_plan);

 if(success){
   ROS_INFO("Planning OK");
 }else{
   ROS_INFO("Error");
 }

 group.move();
*/

 ROS_INFO("Attach the object to the robot");
 group.attachObject(collision_object.id);
 sleep(8.0);

 ROS_INFO("Detach the object from the robot");
 group.detachObject(collision_object.id);
 sleep(8.0);

 ROS_INFO("Remove the object from the world");
 std::vector<std::string> object_ids;
 object_ids.push_back(collision_object.id);
 planning_scene_interface.removeCollisionObjects(object_ids);
 /* Sleep to give Rviz time to show the object is no longer there. */
 sleep(4.0);
 
 ros::waitForShutdown();
}
