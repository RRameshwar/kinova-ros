#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Vector3.h"

#include <sstream>
#include <cmath>

#define PI 3.14159265

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("kinova_glove_control", 1000);

  ros::Rate loop_rate(10);

  int count = 0;

  float num;

  std_msgs::Float32MultiArray pos;
  pos.data.push_back(270);
  pos.data.push_back(270);
  pos.data.push_back(125);
  pos.data.push_back(106);
  pos.data.push_back(103);
  pos.data.push_back(79.71);
  pos.data.push_back(0.0);

  while (ros::ok())
  {

    num = 30*sin(count*3.14/20)+255;

    pos.data[1] = num;

    chatter_pub.publish(pos);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}