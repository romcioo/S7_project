#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <sstream>
#include <iostream>
#include <random>
//#include <time>
// #include "webots/Robot.h"

/*Declare*/
void objectiveCallback(const geometry_msgs::Vector3::ConstPtr& msg);
void globalCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
geometry_msgs::Vector3Stamped addNoise(geometry_msgs::Vector3Stamped point);
std_msgs::Bool reachedQ(geometry_msgs::Vector3 objective, geometry_msgs::Vector3 position);

extern geometry_msgs::Vector3Stamped global_position = geometry_msgs::Vector3Stamped();
extern geometry_msgs::Vector3 objective = geometry_msgs::Vector3();

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "planner");
  ros::NodeHandle n;
  ros::Publisher heading_pub = n.advertise<geometry_msgs::Quaternion>("heading", 1000);
  ros::Publisher visited_pub = n.advertise<geometry_msgs::Vector3Stamped>("path", 1000);
  ros::Publisher reached_pub = n.advertise<std_msgs::Bool>("reachPoint", 1000);
  // ros::Publisher position_shifted_pub = n.advertise<geometry_msgs::Vector3>("robot_pose_shifted", 1000);

  ros::Subscriber objective_sub = n.subscribe("goal", 1000, objectiveCallback);
  ros::Subscriber global_position_sub = n.subscribe("robot_pose", 1000, globalCallback);

  ros::Rate loop_rate(10);

  // Initialize
  /*global_position.vector.x = double(0);
  global_position.vector.y = double(0);
  global_position.vector.z = double(0);*/

  geometry_msgs::Quaternion heading_msg;
  geometry_msgs::Vector3Stamped position;
  std_msgs::Bool reached;

  while (ros::ok()) {
    position = addNoise(global_position);
    visited_pub.publish(position);

    reached = reachedQ(objective, position.vector);
    reached_pub.publish(reached);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}

void objectiveCallback(const geometry_msgs::Vector3::ConstPtr& msg){
  objective = *msg;
}

void globalCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
  /*geometry_msgs::Vector3 vector = msg->vector;
  std_msgs::Header header = msg->header;

  double posx = vector.x;
  double posy = vector.y;
  double posz = vector.z;

  //global_position = geometry_msgs::Vector3(posx,posy,posz);
  //geometry_msgs::Vector3

  global_position.vector.x = posx;
  global_position.vector.y = posy;
  global_position.vector.z = posz;*/
  global_position = *msg;
}

geometry_msgs::Vector3Stamped addNoise(geometry_msgs::Vector3Stamped point) {
  // double stddev;
  // const double mean = 0.0;
  // std::default_random_engine generator;

  geometry_msgs::Vector3Stamped vector = point;
  // ros::Time time = point->header.stamp;

  // std::normal_distribution<double> dist(mean, stddev);

  return vector;
}

std_msgs::Bool reachedQ(geometry_msgs::Vector3 objective, geometry_msgs::Vector3 position) {
  float error = .25;

  float x = objective.x - position.x;
  float y = objective.y - position.y;
  float z = objective.z - position.z;

  std_msgs::Bool reached = std_msgs::Bool();
  reached.data = error > std::sqrt(pow(x,2) + pow(y,2) + pow(z,2));
  return reached;
}
