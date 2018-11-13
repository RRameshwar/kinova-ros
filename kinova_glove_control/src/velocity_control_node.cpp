#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Vector3.h"
#include "kinova_msgs/JointAngles.h"
#include "kinova_msgs/JointVelocity.h"

#include <sstream>
#include <cmath>

#include <ros/console.h>

#define PI 3.14159265

class JacoVelocityControl {
	public:
		ros::NodeHandle n;
		
		ros::Subscriber imu_calibrator;

		ros::Subscriber curr_pos_sub;

		ros::Subscriber next_pos_sub;

		ros::Publisher joint_vel;

		kinova_msgs::JointVelocity joint_velocity_msg;

		std::vector<float> curr_pos;
		std::vector<float> next_pos;
		std::vector<float> vels;

		float P;
		float D;
		float derivator;

		ros::Timer timer_pub_joint_vel_;
	    


	JacoVelocityControl(){

		curr_pos_sub = n.subscribe("/j2n6s300_driver/out/joint_angles", 1000, &JacoVelocityControl::CurrPosCB, this);
		next_pos_sub = n.subscribe("kinova_glove_control", 1000, &JacoVelocityControl::NextPosCB, this);

  		joint_vel = n.advertise<kinova_msgs::JointVelocity>("j2n6s300_driver/in/joint_velocity", 1000);

  		timer_pub_joint_vel_ = n.createTimer(ros::Duration(0.01), &JacoVelocityControl::pub_joint_vel, this, false, false);


        joint_velocity_msg.joint1 = 0;
        joint_velocity_msg.joint2 = 0;
        joint_velocity_msg.joint3 = 0;
        joint_velocity_msg.joint4 = 0;
        joint_velocity_msg.joint5 = 0;
        joint_velocity_msg.joint6 = 0;
        joint_velocity_msg.joint7 = 0;

        P = 2.4;
        D = 0.1;
        derivator = 0;

        for (int i = 0; i<8; i++){
        	vels.push_back(0);
        }

        //270.0, 270.0, 185.0, 6.0, 18.0, 300
        curr_pos.push_back(270);
        curr_pos.push_back(180);
        curr_pos.push_back(90);
        curr_pos.push_back(6);
        curr_pos.push_back(18);
        curr_pos.push_back(300);
        curr_pos.push_back(0.0);

        next_pos.push_back(270);
        next_pos.push_back(180);
        next_pos.push_back(90);
        next_pos.push_back(6);
        next_pos.push_back(18);
        next_pos.push_back(300);
        next_pos.push_back(0.0);

        timer_pub_joint_vel_.start();
	}

	void pub_joint_vel(const ros::TimerEvent&){
		joint_velocity_msg.joint1 = vels[0];
        joint_velocity_msg.joint2 = vels[1];
        joint_velocity_msg.joint3 = vels[2];
        joint_velocity_msg.joint4 = vels[3];
        joint_velocity_msg.joint5 = vels[4];
        joint_velocity_msg.joint6 = vels[5];
        joint_velocity_msg.joint7 = vels[6];

        if (ros::ok())
        	joint_vel.publish(joint_velocity_msg);

	}

	void CurrPosCB(const kinova_msgs::JointAngles& msg){
		curr_pos[0] = msg.joint1;
		curr_pos[1] = msg.joint2;
		curr_pos[2] = msg.joint3;
		curr_pos[3] = msg.joint4;
		curr_pos[4] = msg.joint5;
		curr_pos[5] = msg.joint6;
		curr_pos[6] = msg.joint7;

		//std::cout << curr_pos.data[2] << std::endl;
	}

	void NextPosCB(const std_msgs::Float32MultiArray& msg){
		for (int i = 0; i < 8; i++){
			next_pos[i] = msg.data[i];
		}
	}

	float constrain(float a, float lo, float hi){
		if (a <= lo){
			return lo;
		}
		else if (a >= hi){
			return hi;
		}
		else{
			return a;
		}
	}

	void PID(){
		float error;
		float P_value;
		float D_value;
		
		float vel;

		for (int i = 0; i < 8; i++){
			error = curr_pos[i] - next_pos[i];
			
			if (abs(error) < 0.5){
				error = 0;
			}

			if (i == 5){
				P_value = 3.5*error;
			}
			else{
				P_value = P*error;
			}

			D_value = D * (error - derivator);
			derivator = error;

			
			vel = P_value + D_value;
			//vel = constrain(vel, -50, 50);

			std::cout << "Joint: " << i << "Current: " << curr_pos[i] << " Next: " << next_pos[i] << " Error: " << error << " Vel: " << vel << std::endl;
			vels[i] = -1*vel;
		}
	}


};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test");

	JacoVelocityControl jc;

	ros::Rate update_rate(100);

	int count = 0;
	while (jc.n.ok())
	{
		ros::spinOnce();

		jc.PID();

		update_rate.sleep();
		++count;
	}

	return 0;
}