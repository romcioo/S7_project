/*Include*/
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <sstream>
#include <iostream>
#include <random>
#include <math.h>
/*!Include!*/

/*Declare*/
void objectiveCallback(const geometry_msgs::Vector3::ConstPtr& msg);
extern geometry_msgs::Vector3 objective = geometry_msgs::Vector3();
void globalCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
extern geometry_msgs::Vector3Stamped global_position = geometry_msgs::Vector3Stamped();
void imuCallback(const geometry_msgs::Vector3::ConstPtr& msg);
extern geometry_msgs::Vector3 imu = geometry_msgs::Vector3();

geometry_msgs::Vector3Stamped addNoise(geometry_msgs::Vector3Stamped point);
std_msgs::Bool reachedQ(geometry_msgs::Vector3 objective, geometry_msgs::Vector3 position);
void head(double initial[], geometry_msgs::Vector3Stamped position, geometry_msgs::Vector3 &heading, ros::Publisher diff_pub, ros::Publisher diff_rot_pub);
void multiply(double trans[3][3], double point[3], double result[3]);
void substract(geometry_msgs::Vector3 vec1, geometry_msgs::Vector3 vec2, double result[3]);
/*!Declare!*/


int main(int argc, char **argv) {
  ros::init(argc, argv, "planner");
  ros::NodeHandle n;
  ros::Publisher heading_pub = n.advertise<geometry_msgs::Vector3>("heading", 1000);
  ros::Publisher visited_pub = n.advertise<geometry_msgs::Vector3Stamped>("path", 1000);
  ros::Publisher reached_pub = n.advertise<std_msgs::Bool>("reachPoint", 1000);

  ros::Subscriber objective_sub = n.subscribe("goal", 1000, objectiveCallback);
  ros::Subscriber global_position_sub = n.subscribe("robot_pose", 1000, globalCallback);
  ros::Subscriber imu_sub = n.subscribe("imuValues", 1000, imuCallback);

  ros::Rate loop_rate(1);

  geometry_msgs::Vector3Stamped position;
  std_msgs::Bool reached;
  geometry_msgs::Vector3 heading = geometry_msgs::Vector3();
  double initial_rotation[3] = {0,0,0};

  while (ros::ok()) {
    position = addNoise(global_position);
    visited_pub.publish(position);

    reached = reachedQ(objective, position.vector);
    reached_pub.publish(reached);

    head(initial_rotation, position, heading, diff_pub, diff_rot_pub);
    heading_pub.publish(heading);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}

void objectiveCallback(const geometry_msgs::Vector3::ConstPtr& msg){
  objective = *msg;
}

void globalCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
  global_position = *msg;
}

void imuCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
  imu = *msg;
}

geometry_msgs::Vector3Stamped addNoise(geometry_msgs::Vector3Stamped point) {
  geometry_msgs::Vector3Stamped vector = point;

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

void head(double initial[], geometry_msgs::Vector3Stamped position, geometry_msgs::Vector3 &heading, ros::Publisher diff_pub, ros::Publisher diff_rot_pub) {
  double diff[3];
  double diff_rot[3];
  geometry_msgs::Vector3 diffV;
  geometry_msgs::Vector3 diff_rotV;

  double roll = imu.x + initial[0];
  double yaw = imu.y + initial[1];
  double pitch = imu.z + initial[2];

  double rotation_matrix[3][3] = {{cos(roll)*cos(pitch) - sin(roll)*cos(yaw)*sin(pitch),
    -cos(roll)*sin(pitch) - sin(roll)*cos(yaw)*cos(pitch),
    sin(roll)*sin(yaw)},
    {sin(roll)*cos(pitch) + cos(roll)*cos(yaw)*sin(pitch),
    -sin(roll)*sin(pitch) + cos(roll)*cos(yaw)*cos(pitch),
    -cos(roll)*sin(yaw)},
    {sin(yaw)*sin(pitch), sin(yaw)*cos(pitch), cos(yaw)}};

  substract(objective, position.vector, diff);
  multiply(rotation_matrix, diff, diff_rot);

  heading.y = atan(diff_rot[1]/diff_rot[0]);
  heading.x = std::sqrt(pow(diff_rot[0],2) + pow(diff_rot[1],2) + pow(diff_rot[2],2));
}

void multiply(double trans[3][3], double point[3], double result[3]) {
  for (int i=0; i<3; i++) {
    double sum = 0;
    for (int j=0; j<3; j++) {
      sum += trans[i][j]*point[j];
    }
    result[i] = sum;
  }
}

void substract(geometry_msgs::Vector3 vec1, geometry_msgs::Vector3 vec2, double result[3]) {
  result[0] = vec1.x - vec2.x;
  result[1] = vec1.y - vec2.y;
  result[2] = vec1.z - vec2.z;
}
