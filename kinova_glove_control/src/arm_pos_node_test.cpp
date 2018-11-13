#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"

#include <sstream>
#include <cmath>

#include <armadillo>

#define pi 3.14159265

using namespace arma;

class JacoFingerControl {
public:
	ros::NodeHandle n;

	ros::Subscriber subFingers;

	ros::Subscriber subWrist;
	ros::Subscriber subForearm;
	ros::Subscriber subUpperarm;

	ros::Subscriber imuCalibrator;
	ros::Subscriber fingerCalibrator;

	ros::Publisher joint_pub;
	ros::Publisher wrist_pub;

	std_msgs::Float32MultiArray pos;

	std::list<float> thumbAngles;
	std::list<float> pointerAngles;
	std::list<float> middleAngles;

	std::vector<float> jointHomes;

	std_msgs::Float32MultiArray IMUoffsets;
	std_msgs::Float32MultiArray fingerOffsets;

	bool IMUcalibrated;
	bool fingerCalibrated;

	float upperArmx;
	float upperArmy;
	float upperArmz;

	float foreArmx;
	float foreArmy;
	float foreArmz;
	
	float Wristx;
	float Wristy;
	float Wristz;

	float foreArmx_raw;
	float foreArmy_raw;
	float foreArmz_raw;
	
	float Wristx_raw;
	float Wristy_raw;
	float Wristz_raw;

	geometry_msgs::Vector3 wrist_value;

	std::vector<float> rangeJoint0;
	std::vector<float> rangeJoint1;
	std::vector<float> rangeJoint2;

	std::vector<int> x1range;
	std::vector<int> y1range;
	std::vector<int> y2range;
	std::vector<int> x12range;

	float imuPrev;



	JacoFingerControl(){
		
		//subFingers = n.subscribe("fingersPos", 1000, &JacoFingerControl::FingersPosCallback, this);

        subWrist = n.subscribe("imuWrist", 1000, &JacoFingerControl::WristPosCallback, this);
        subForearm = n.subscribe("imuForearm", 1000, &JacoFingerControl::ForearmPosCallback, this);
        subUpperarm = n.subscribe("imuUpperarm", 1000,  &JacoFingerControl::UpperarmPosCallback, this);

        imuCalibrator = n.subscribe("imuCalibrator", 1000, &JacoFingerControl::IMUCalibrateCallback, this);

        // 179, 270, 90, 106.0, 103.0, 79.0, 0.0 NEW HOME
        
        pos.data.push_back(270);
        pos.data.push_back(270);
        pos.data.push_back(185);
        pos.data.push_back(6);
        pos.data.push_back(18);
        pos.data.push_back(300);
        pos.data.push_back(0.0);

        pos.data.push_back(0);
        pos.data.push_back(0);
        pos.data.push_back(0);

        wrist_value.x = 0;
        wrist_value.y = 0;
        wrist_value.z = 0;
        

  		joint_pub = n.advertise<std_msgs::Float32MultiArray>("/kinova_glove_control", 1000);
  		wrist_pub = n.advertise<geometry_msgs::Vector3>("/wrist_value", 1000);


        //newhome: [270.0, 270.0, 185.0, 6.0, 18.0, 300]
        jointHomes.push_back(270);
        jointHomes.push_back(270);
        jointHomes.push_back(185);
        jointHomes.push_back(6);
        jointHomes.push_back(18);
        jointHomes.push_back(300);
        jointHomes.push_back(0.0);

        
		rangeJoint0.push_back(250); //ceiling
		rangeJoint0.push_back(290); //table
		

		rangeJoint1.push_back(190); //towards door
		rangeJoint1.push_back(290); //at sink

		rangeJoint2.push_back(50); //elbow in
		rangeJoint2.push_back(185); //elbow out
		

		x1range.push_back(80); //towards door
		x1range.push_back(-20); //towards sink
		
		
		y1range.push_back(-50); // ceiling
		y1range.push_back(50); //table

		//difference between 1x and 2x
		x12range.push_back(-70); //elbow in
		x12range.push_back(0); //elbow out
		

        IMUcalibrated = false;
        fingerCalibrated = false;

        for (int i = 0; i<9; i++){
        	IMUoffsets.data.push_back(0.0);
        }

        for (int i = 0; i<3; i++){
        	fingerOffsets.data.push_back(0.0);
        }

        imuPrev = 0;
	}

	float calculateAvg(std::list<float> list)
	{
	    double avg = 0;
	    std::list<float>::iterator it;
	    for(it = list.begin(); it != list.end(); it++) avg += *it;
	    avg /= list.size();
		return avg;
	}

	float map(float value, float home, std::vector<int> sensor_range, std::vector<float> joint_range){ 

		if(value == 0){
			return home;
			std::cout << "AM HOME" << std::endl;
		}
		float num = (((value-sensor_range[0])/(sensor_range[1]-sensor_range[0]))*(joint_range[1]-joint_range[0]))+joint_range[0];

		if (num > joint_range[1])
			{
				return  joint_range[1];
			}
		else if (num < joint_range[0])
			{
				return  joint_range[0];
			}
		else
			{
				return num;
			}
	}

	float fingerMap(int value, float max, float min){
		float angle = (value - max)/-(max-min);
		
		if(angle < 0.001){
			angle = 0;
		}
		
		else if(angle >= 1){
		  angle = 1;
		}

		return angle;
	}

	void IMUCalibrateCallback(const std_msgs::Float32MultiArray& msg){
		IMUoffsets.data = msg.data;
		IMUcalibrated = 1;
	}

	void FingerCalibrateCallback(const std_msgs::Float32MultiArray& msg){
		fingerOffsets.data = msg.data;
		fingerCalibrated = 1;
	}

	void WristPosCallback(const geometry_msgs::Vector3& msg){
		
		Wristx_raw = msg.x;
		Wristy_raw = msg.y;
		Wristz_raw = msg.z;
		
		float x = msg.x - IMUoffsets.data[6];
		float y = msg.y - IMUoffsets.data[7];
		float z = msg.z - IMUoffsets.data[8];
		float diff;

		//y controls joint 7, 90 twist right, -30 twist left
		//z controls wrist servo, -70 turn up, 60 turn down

		if (z > 150){
			z = z-360;
		}


		if (x <= -30){
			x = x+360;
		}

		diff = (foreArmy) - (Wristz);

		if (diff < -180){
			diff = diff + 360;
		}
		// if (diff > 180){
		// 	diff = diff - 360;
		// }
		//std::cout << "IMU PREV: " << imuPrev << std::endl;
		Wristx = x;
		Wristy = y;
		Wristz = z;
		
		//std::cout << (foreArmy) << " "   << (Wristz) << " " << (diff) << std::endl;
		
	}

	void ForearmPosCallback(const geometry_msgs::Vector3& msg){
		foreArmx_raw = msg.x;
		foreArmy_raw = msg.y;
		foreArmz_raw = msg.z;

		float x = msg.x - IMUoffsets.data[3];
		float y = msg.y - IMUoffsets.data[4];
		float z = msg.z - IMUoffsets.data[5];

		
		float x0 = upperArmx;
		float y0 = upperArmy;

		if (x >= 180){
			x = x - 360;
		}

		if (x <= -180){
			x = x + 360;
		}

		if (x0 >= 180){
			x0 = x0 - 360;
		}

		if (x0 < -180){
			x0 = x0 + 360;
		}

		foreArmx = x;
		foreArmy = y;
		foreArmz = z;

		float num = (x - x0) /*+ (y - y0)*/;

		//std::cout << x << " "   << x0 << " " << num << std::endl;

		if (num < -180){
			num = num + 360;
		}

		float ans_joint2 = map(num, jointHomes[2], x12range, rangeJoint2);

		pos.data[2]= ans_joint2;
	}


	void UpperarmPosCallback(const geometry_msgs::Vector3& msg){
		float x = msg.x - IMUoffsets.data[0];
		float y = msg.y - IMUoffsets.data[1];
		float z = msg.z - IMUoffsets.data[2] + 180;

		//std::cout << IMUoffsets.data[0] << std::endl;

		if (y > 180){
			y = y - 360;
		}
		if (x > 180){
			x = x - 360;
		}
		if (x < -180){
			x = x+360;
		}

		upperArmx = x;
		upperArmy = y;
		upperArmz = z;

		//std::cout << y << "   " << x << std::endl;

		float ans_joint0 = map((y), jointHomes[0], y1range, rangeJoint0);
		
		float ans_joint1 = map((x), jointHomes[1], x1range, rangeJoint1);

		pos.data[0]=ans_joint0;
		pos.data[1]=ans_joint1;
	}
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "asdf");

  JacoFingerControl jc;

  ros::Rate loop_rate(20);

  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();

    if (jc.IMUcalibrated){
    	jc.joint_pub.publish(jc.pos);
    	jc.WristCalc();
    	jc.wrist_pub.publish(jc.wrist_value);

    	std::cout << std::fixed;
    	std::cout << std::setprecision(2);
    	//std::cout << "Wrist " <<  jc.Wristx << "     " << jc.Wristy << "     " << jc.Wristz << std::endl;
    	//std::cout << "Forearm " <<  jc.foreArmx << "   " << jc.foreArmy << "  " << jc.foreArmz << std::endl;

    }

    loop_rate.sleep();
    ++count;
  }

  return 0;
}