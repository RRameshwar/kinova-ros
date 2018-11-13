#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32MultiArray.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");

 
  ros::NodeHandle n;

  geometry_msgs::Vector3 msg;
  msg.x = 1;
  msg.y = 2;
  msg.z = 3;

  std_msgs::Float32MultiArray msg2;
  msg2.data.push_back(0);
  msg2.data.push_back(1);
  msg2.data.push_back(2);
  msg2.data.push_back(3);

 
  ros::Publisher upper_pub = n.advertise<geometry_msgs::Vector3>("imuUpperarm", 1000);
  ros::Publisher fore_pub = n.advertise<geometry_msgs::Vector3>("/imuForearm", 1000);
  ros::Publisher wrist_pub = n.advertise<geometry_msgs::Vector3>("/imuWrist", 1000);
  
  ros::Publisher fings_pub = n.advertise<std_msgs::Float32MultiArray>("/fingersPos", 1000);

  ros::Rate loop_rate(10);

 
  int count = 0;
  while (ros::ok())
  {

    upper_pub.publish(msg);
    fore_pub.publish(msg);
    wrist_pub.publish(msg);

    fings_pub.publish(msg2);


    
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}