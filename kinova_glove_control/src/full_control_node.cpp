#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Vector3.h"

#include <sstream>
#include <cmath>

#define PI 3.14159265

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

	sensor_msgs::JointState pos;

	std::list<float> thumbAngles;
	std::list<float> pointerAngles;
	std::list<float> middleAngles;

	std::vector<float> jointHomes;

	std_msgs::Float32MultiArray IMUoffsets;
	std_msgs::Float32MultiArray fingerOffsets;

	bool IMUcalibrated;
	bool fingerCalibrated;

	float imu3x;
	float imu3y;
	float imu2x;
	float imu2y;

	JacoFingerControl(){
		
		subFingers = n.subscribe("fingersPos", 1000, &JacoFingerControl::FingersPosCallback, this);

        subWrist = n.subscribe("imu1", 1000, &JacoFingerControl::WristPosCallback, this);
        subForearm = n.subscribe("imu2", 1000, &JacoFingerControl::ForearmPosCallback, this);
        subUpperarm = n.subscribe("imu3", 1000,  &JacoFingerControl::UpperarmPosCallback, this);

        imuCalibrator = n.subscribe("imuCalibrator", 1000, &JacoFingerControl::IMUCalibrateCallback, this);
        fingerCalibrator = n.subscribe("fingerCalibrator", 1000, &JacoFingerControl::FingerCalibrateCallback, this);


        pos.name.push_back("jaco_arm_0_joint");
        pos.name.push_back("jaco_arm_1_joint");
        pos.name.push_back("jaco_arm_2_joint");
        pos.name.push_back("jaco_arm_3_joint");
        pos.name.push_back("jaco_arm_4_joint");
        pos.name.push_back("jaco_arm_5_joint");

        pos.name.push_back("jaco_finger_joint_0");
        pos.name.push_back("jaco_finger_joint_2");
        pos.name.push_back("jaco_finger_joint_4");

        /*pos.position.push_back(-1.7299817256187797);
        pos.position.push_back(-1.7326067624496215 + (3.1415/2));
        pos.position.push_back(0.7036460166034555);
        pos.position.push_back(-0.8174465286212945);
        pos.position.push_back(1.5064013197669412);
        pos.position.push_back(3.135943485311694);*/

        pos.position.push_back(-0.83 - (3.1415/4));
        pos.position.push_back(0.0);
        pos.position.push_back(-1.6);
        pos.position.push_back(0.03 + (3.14/2));
        pos.position.push_back(-3*3.14/2);
        pos.position.push_back(-0.05);

        pos.position.push_back(0);
        pos.position.push_back(0);
        pos.position.push_back(0);
        

  		joint_pub = n.advertise<sensor_msgs::JointState>("/jaco/joint_control", 1000);

  		/*jointHomes.push_back(-1.7299817256187797);
        jointHomes.push_back(-1.7326067624496215 + (3.1415/2));
        jointHomes.push_back(0.7036460166034555);
        jointHomes.push_back(-0.8174465286212945);
        jointHomes.push_back(1.5064013197669412);
        jointHomes.push_back(3.135943485311694);*/

        jointHomes.push_back(-0.83 - (3.1415/4));
        jointHomes.push_back(0.0);
        jointHomes.push_back(-1.6);
        jointHomes.push_back(0.03 + (3.14/2));
        jointHomes.push_back(-3*3.14/2);
        jointHomes.push_back(-0.05);

        IMUcalibrated = false;
        fingerCalibrated = false;

        for (int i = 0; i<9; i++){
        	IMUoffsets.data.push_back(0.0);
        }

        for (int i = 0; i<3; i++){
        	fingerOffsets.data.push_back(0.0);
        }

        imu3x = 0;
        imu3y = 0;
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

		if(value == 0){return home;}
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
		float x = msg.x - IMUoffsets.data[0];
		float y = msg.y - IMUoffsets.data[1];
		float z = msg.z - IMUoffsets.data[2];

		std::vector<int> xrange;
		xrange.push_back(-25);
		xrange.push_back(25);

		std::vector<int> yrange;
		yrange.push_back(-50);
		yrange.push_back(50);

		std::vector<int> zrange;
		zrange.push_back(45);
		zrange.push_back(-45);

		std::vector<float> xJointrange;
		xJointrange.push_back(jointHomes[5]-1);
		xJointrange.push_back(jointHomes[5]+1);

		std::vector<float> yJointrange;
		yJointrange.push_back(jointHomes[6]-1);
		yJointrange.push_back(jointHomes[6]+1);

		std::vector<float> zJointrange;
		zJointrange.push_back(-3.14);
		zJointrange.push_back(-1.68);

		float ansz = map(z, jointHomes[4], zrange, zJointrange);

		//pos.position[4]=ansz;
	}

	void ForearmPosCallback(const geometry_msgs::Vector3& msg){
		float x = msg.x - IMUoffsets.data[3];
		float y = msg.y - IMUoffsets.data[4];

		imu2x = x;
		imu2y = y;

		float x0 = imu3x;
		float y0 = imu3y;

		if (x >= 180){
			x = x - 360;
		}

		if (x0 >= 180){
			x0 = x0 - 360;
		}

		std::vector<int> yrange;
		yrange.push_back(-3); //arm up
		yrange.push_back(-65); //arm down

		std::vector<float> rangeJoint2;
		rangeJoint2.push_back(jointHomes[2]-1);
		rangeJoint2.push_back(jointHomes[2]+2.7);

		float num = (x - x0) /*+ (y - y0)*/;

		//std::cout << num << std::endl;

		float ans_joint2 = map(num, jointHomes[2], yrange, rangeJoint2);

		pos.position[2]= ans_joint2;
	}

	std::vector<float> forwardKinematics(float x1, float y1, float x2, float y2){
		float a1 = 280;
		float a2 = 260;

		x1 = x1 * PI/180;
		y1 = y1 * PI/180;
		x2 = x2 * PI/180;
		y2 = y2 * PI/180;

 		float y =  a1*cos(x1)*cos(y1) + a2*cos(x2)*cos(y2);
 		float z =  (a1*sin(y1) + a2*sin(y2));

 		std::cout << "Y: " << y << " Z: " << z << std::endl; 

 		std::vector<float> v;
 		v.push_back(y);
 		v.push_back(z);

 		return v;

	}

	void UpperarmPosCallback(const geometry_msgs::Vector3& msg){
		float x = msg.x - IMUoffsets.data[0];
		float y = msg.y - IMUoffsets.data[1];
		float z = msg.z - IMUoffsets.data[2] + 180;

		imu3x = x;
		imu3y = y;

		if (y > 180){
			y = y - 360;
		}

		//pos.position[1] = imu3x;

		//pos.position[2] = imu2x - imu3x;
		//pos.position[0] = atan2(physicalZ, physicalY)

		std::vector<int> xrange;
		xrange.push_back(-35); //40, 130
		xrange.push_back(35);

		std::vector<int> yrange;
		yrange.push_back(35); //40, 130
		yrange.push_back(-35);

		std::vector<int> yzrange;

		std::vector<float> rangeJoint0;
		rangeJoint0.push_back(jointHomes[0]-1);
		rangeJoint0.push_back(jointHomes[0]+1);

		std::vector<float> rangeJoint1;
		rangeJoint1.push_back(jointHomes[1]-1.3);
		rangeJoint1.push_back(jointHomes[1]+.7);

		/*std::vector<float> cart = forwardKinematics(x, y, imu2x, imu2y);
		float num = atan2(cart[1], cart[0]);*/

		float ans_joint0 = map((y), jointHomes[0], yrange, rangeJoint0);
		float ans_joint1 = map((xrange[1] - x), jointHomes[1], xrange, rangeJoint1);

		pos.position[0]=ans_joint0;
		pos.position[1]=ans_joint1;
	}

	void FingersPosCallback(const geometry_msgs::Vector3& msg){
	  int thumb =  msg.x;
	  int pointer = msg.y;
	  int middle = msg.z;

	  float ansThumb = fingerMap(thumb, fingerOffsets.data[0], fingerOffsets.data[3]);
	  float ansPointer = fingerMap(pointer, fingerOffsets.data[1], fingerOffsets.data[4]);
	  float ansMiddle = fingerMap(middle, fingerOffsets.data[2], fingerOffsets.data[5]);

	  pos.position[7] = ansThumb;
	  pos.position[6] = ansPointer;
	  pos.position[8] = ansMiddle;

	}

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "asdf");

  JacoFingerControl jc;

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();

    //jc.joint_pub.publish(jc.pos);

    if (jc.IMUcalibrated and jc.fingerCalibrated){
    	jc.joint_pub.publish(jc.pos);
    	//std::cout << "publishing" << std::endl;
    }

    loop_rate.sleep();
    ++count;
  }

  return 0;
}