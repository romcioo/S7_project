/*Include*/
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
// #include "geometry_msgs/Vector3Stamped.h"
#include <sstream>
#include <iostream>
#include <random>
#include <math.h>
#include <stdlib.h>
/*!Include!*/

/*Declare*/
void objectiveCallback(const geometry_msgs::Vector3::ConstPtr& msg);
extern geometry_msgs::Vector3 objective = geometry_msgs::Vector3();
void globalCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
extern geometry_msgs::Quaternion global_position = geometry_msgs::Quaternion();
void imuCallback(const geometry_msgs::Vector3::ConstPtr& msg);
extern geometry_msgs::Vector3 imu = geometry_msgs::Vector3();

// geometry_msgs::Vector3Stamped addNoise(geometry_msgs::Vector3Stamped point);
std_msgs::Bool reachedQ(geometry_msgs::Vector3 objective, geometry_msgs::Quaternion position);
void head(double initial[], geometry_msgs::Quaternion position, geometry_msgs::Vector3 &heading);
void multiply(double trans[3][3], double point[3], double result[3]);
void subtract(geometry_msgs::Vector3 vec1, geometry_msgs::Vector3 vec2, double result[3]);
/*!Declare!*/


int main(int argc, char **argv) {
  ros::init(argc, argv, "planner"); // topic name remappings
  ros::NodeHandle n; // Node

  /*Publishers*/
  ros::Publisher heading_pub = n.advertise<geometry_msgs::Vector3>("heading", 1000);
  // ros::Publisher visited_pub = n.advertise<geometry_msgs::Vector3Stamped>("path", 1000);
  ros::Publisher reached_pub = n.advertise<std_msgs::Bool>("reachPoint", 1);
  // ros::Publisher absolute_imu_pub = n.advertise<geometry_msgs::Vector3>("absoluteIMU", 1000);
  /*!Publishers!*/

  /*Subscribers*/
  ros::Subscriber objective_sub = n.subscribe("goal", 1000, objectiveCallback);
  ros::Subscriber global_position_sub = n.subscribe("path", 1000, globalCallback);
  ros::Subscriber imu_sub = n.subscribe("imuValues", 1000, imuCallback);
  /*!Subscribers!*/

  ros::Rate loop_rate(50); // time delay in Hz (10Hz)

  /*Declare*/
  // geometry_msgs::Vector3Stamped position;
  std_msgs::Bool reached;
  geometry_msgs::Vector3 heading = geometry_msgs::Vector3();
  double initial_rotation[3] = {0,0,0};
  /*!Declare!*/

  while (ros::ok()) {
    // position = addNoise(global_position);
    // visited_pub.publish(global_position);

    reached = reachedQ(objective, global_position);
    reached_pub.publish(reached);

    head(initial_rotation, global_position, heading);
    heading_pub.publish(heading);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

/*Functions*/

/*Get ROS information into main*/
void objectiveCallback(const geometry_msgs::Vector3::ConstPtr& msg){
  objective = *msg;
}

void globalCallback(const geometry_msgs::Quaternion::ConstPtr& msg) {
  global_position = *msg;
}

void imuCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
  imu = *msg;
}
/*!Get ROS information into main!*/

// geometry_msgs::Vector3Stamped addNoise(geometry_msgs::Vector3Stamped point) { // add noise to the position
//   geometry_msgs::Vector3Stamped vector = point;
//
//   return vector;
// }

std_msgs::Bool reachedQ(geometry_msgs::Vector3 objective, geometry_msgs::Quaternion position) { // Check if the robot has arrived to the current goal
  float error = .25; // radious to accept reached

  /*Difference on each coordinate*/
  float x = objective.x - position.x;
  float y = objective.y - position.y;
  float z = objective.z - position.z;
  /*!Difference on each coordinate!*/

  std_msgs::Bool reached = std_msgs::Bool();
  reached.data = error > std::sqrt(pow(y,2) + pow(z,2));
  return reached;
}

void head(double initial[], geometry_msgs::Quaternion position, geometry_msgs::Vector3 &heading) { // calculate the turning angle and distance to the goal
  /*Declare*/
  double diff[3];
  double diff_rot[3];
  geometry_msgs::Vector3 pos;
  double angle;
  /*!Declare!*/

  double roll = imu.x + initial[0];
  double pitch = imu.y + initial[1];
  double yaw = imu.z + initial[2];
  double pi = 2*acos(0.0);

  geometry_msgs::Vector3 pub_imu;
  pub_imu.x = roll;
  pub_imu.y = yaw;
  pub_imu.z = pitch;
  // global_imu_pub.publish(pub_imu);

  /*Matrix to rotate on Euler angles*/
  double rotation_matrix[3][3] = {{cos(roll)*cos(pitch) - sin(roll)*cos(yaw)*sin(pitch),
    -cos(roll)*sin(pitch) - sin(roll)*cos(yaw)*cos(pitch),
    sin(roll)*sin(yaw)},
    {sin(roll)*cos(pitch) + cos(roll)*cos(yaw)*sin(pitch),
    -sin(roll)*sin(pitch) + cos(roll)*cos(yaw)*cos(pitch),
    -cos(roll)*sin(yaw)},
    {sin(yaw)*sin(pitch), sin(yaw)*cos(pitch), cos(yaw)}};
    
    /*double rotation_matrix[3][3] = {{cos(yaw)*cos(pitch) - cos(roll)*sin(yaw)*sin(pitch),
  cos(yaw)*sin(pitch) + cos(roll)*cos(pitch)*sin(yaw),
  sin(yaw)*sin(roll)},
  {-sin(yaw)*cos(pitch) - cos(roll)*sin(pitch)*cos(yaw),
  -sin(yaw)*sin(pitch) + cos(roll)*cos(pitch)*cos(yaw),
  cos(yaw)*sin(roll)},
  {sin(roll)*sin(pitch),
  -sin(roll)*cos(pitch),
  cos(roll)}};*/
  /*!Matrix to rotate on Euler angles!*/

  pos = geometry_msgs::Vector3();
  pos.x = position.x;
  pos.y = position.y;
  pos.z = position.z;

  /*Get vector rotated*/
  subtract(objective, pos, diff);
  multiply(rotation_matrix, diff, diff_rot);
  /*Get vector rotated*/

  angle = atan2(-diff_rot[2],diff_rot[0]); // angle
  /*angle -= pi;
  if (angle < pi) {
  	angle += 2*pi;
  }*/
  heading.y = angle;
  heading.x = std::sqrt(pow(diff[1],2) + pow(diff[2],2));
}

void multiply(double trans[3][3], double point[3], double result[3]) { // Multiply a [3x3] matrix by [3x1]
  for (int i=0; i<3; i++) {
    double sum = 0;
    for (int j=0; j<3; j++) {
      sum += trans[i][j]*point[j];
    }
    result[i] = sum;
  }
}

void subtract(geometry_msgs::Vector3 vec1, geometry_msgs::Vector3 vec2, double result[3]) { // subtract two [3x1] matrix
  result[0] = vec1.x - vec2.x;
  result[1] = vec1.y - vec2.y;
  result[2] = vec1.z - vec2.z;
}
/*!Functions!*/
