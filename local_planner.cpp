#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <sstream>
#include <iostream>
// #include "webots/Robot.h"

/*Declare*/
void objectiveCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
void globalCallback(const geometry_msgs::Vector3::ConstPtr& msg);

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "planner");
  ros::NodeHandle n;
  ros::Publisher heading_pub = n.advertise<geometry_msgs::Quaternion>("heading", 1000);
  ros::Publisher visited_pub = n.advertise<geometry_msgs::Vector3Stamped>("path", 1000);
  ros::Publisher reached_pub = n.advertise<std_msgs::Bool>("reachPoint", 1000);
  ros::Publisher position_shifted_pub = n.advertise<geometry_msgs::Vector3>("robot_pose_shifted", 1000);

  ros::Subscriber objective_sub = n.subscribe("goal", 1000, objectiveCallback);
  ros::Subscriber global_position_sub = n.subscribe("robot_pose", 1000, globalCallback);

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    geometry_msgs::Quaternion heading_msg;



    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}

void objectiveCallback(const geometry_msgs::Quaternion::ConstPtr& msg){
  double posx = msg->x;
}

void globalCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
  double posx = msg->x;
  double posy = msg->y;
  double posz = msg->z;
}
