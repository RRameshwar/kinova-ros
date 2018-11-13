#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

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

	ros::Subscriber subWristQ;
	ros::Subscriber subForearmQ;

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

	float foreArm_qx;
	float foreArm_qy;
	float foreArm_qz;
	float foreArm_qw;
	
	float Wristx;
	float Wristy;
	float Wristz;

	float wrist_qx;
	float wrist_qy;
	float wrist_qz;
	float wrist_qw;

	float foreArmx_raw;
	float foreArmy_raw;
	float foreArmz_raw;
	
	float Wristx_raw;
	float Wristy_raw;
	float Wristz_raw;

	mat foreArm_prev;
	mat wrist_prev;

	std_msgs::Float32 wrist_value;

	std::vector<float> rangeJoint0;
	std::vector<float> rangeJoint1;
	std::vector<float> rangeJoint2;
	std::vector<float> rangeJoint6;
	std::vector<float> wristservoRange;

	std::vector<int> x1range;
	std::vector<int> y1range;
	std::vector<int> y2range;
	std::vector<int> x12range;
	std::vector<int> wristzRange;
	std::vector<int> wristyRange;

	float imuPrev;



	JacoFingerControl(){
		
		//subFingers = n.subscribe("fingersPos", 1000, &JacoFingerControl::FingersPosCallback, this);

        subWrist = n.subscribe("imuWrist", 1000, &JacoFingerControl::WristPosCallback, this);
        subForearm = n.subscribe("imuForearm", 1000, &JacoFingerControl::ForearmPosCallback, this);
        subUpperarm = n.subscribe("imuUpperarm", 1000,  &JacoFingerControl::UpperarmPosCallback, this);

        subWristQ = n.subscribe("imuWristQ", 1000, &JacoFingerControl::WristQPosCallback, this);
        subForearmQ = n.subscribe("imuForearmQ", 1000, &JacoFingerControl::ForearmQPosCallback, this);

        imuCalibrator = n.subscribe("imuCalibrator", 1000, &JacoFingerControl::IMUCalibrateCallback, this);

        // 179, 270, 90, 106.0, 103.0, 79.0, 0.0 NEW HOME
        
        pos.data.push_back(270);
        pos.data.push_back(180);
        pos.data.push_back(90);
        pos.data.push_back(6);
        pos.data.push_back(18);
        pos.data.push_back(300);
        pos.data.push_back(0.0);

        pos.data.push_back(0);
        pos.data.push_back(0);
        pos.data.push_back(0);

        wrist_value.data = 0.0;
        

  		joint_pub = n.advertise<std_msgs::Float32MultiArray>("/kinova_glove_control", 1000);
  		wrist_pub = n.advertise<std_msgs::Float32>("/wrist_value", 1000);


        //newhome: [270.0, 270.0, 185.0, 6.0, 18.0, 300]

        //newhome lesstorque: [270.0, 180.0, 90.0, 6.0, 18.0, 300]

        jointHomes.push_back(270);
        jointHomes.push_back(180);
        jointHomes.push_back(90);
        jointHomes.push_back(6);
        jointHomes.push_back(18);
        jointHomes.push_back(300);
        jointHomes.push_back(90);

        
		rangeJoint0.push_back(250); //ceiling
		rangeJoint0.push_back(290); //table
		

		rangeJoint1.push_back(190); //towards door
		rangeJoint1.push_back(290); //at sink

		rangeJoint2.push_back(50); //elbow in
		rangeJoint2.push_back(185); //elbow out

		rangeJoint6.push_back(230);
		rangeJoint6.push_back(360);

		wristservoRange.push_back(-30); //wrist down
		wristservoRange.push_back(150); //wrist up
		

		x1range.push_back(0); //towards door
		x1range.push_back(125); //towards sink
		
		
		y1range.push_back(40); // ceiling
		y1range.push_back(-40); //table

		//difference between 1x and 2x
		x12range.push_back(20); //elbow in
		x12range.push_back(-35); //elbow out

		wristzRange.push_back(-45); //wrist down
		wristzRange.push_back(55); //wrist up

		wristyRange.push_back(130); //wrist turned counter clockwise
		wristyRange.push_back(85); //wrist turned clockwise
		

        IMUcalibrated = false;
        fingerCalibrated = false;

        for (int i = 0; i<9; i++){
        	IMUoffsets.data.push_back(0.0);
        }

        for (int i = 0; i<3; i++){
        	fingerOffsets.data.push_back(0.0);
        }

        imuPrev = 0;

        foreArm_prev = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        wrist_prev = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
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

	
	void WristQPosCallback(const geometry_msgs::Quaternion& msg){
		wrist_qx = msg.x;
		wrist_qy = msg.y;
		wrist_qz = msg.z;
		wrist_qw = msg.w;
	}


	void WristPosCallback(const geometry_msgs::Vector3& msg){
		
		Wristx_raw = msg.x;
		Wristy_raw = msg.y;
		Wristz_raw = msg.z;
		
		float x = msg.x - IMUoffsets.data[6];
		float y = msg.y - IMUoffsets.data[7];
		float z = msg.z - IMUoffsets.data[8];
		float diff;


		if (z > 150){
			z = z-360;
		}


		if (x <= -30){
			x = x+360;
		}

		Wristx = x;
		Wristy = y;
		Wristz = z;
		
		
		
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

		//std::cout << num << std::endl;

		if (num < -180){
			num = num + 360;
		}

		float ans_joint2 = map(num, jointHomes[2], x12range, rangeJoint2);

		//std::cout << "Joint 2 " << num << "  " << ans_joint2 << std::endl;

		pos.data[2]= ans_joint2;
	}

	void ForearmQPosCallback(const geometry_msgs::Quaternion& msg){
		foreArm_qx = msg.x;
		foreArm_qy = msg.y;
		foreArm_qz = msg.z;
		foreArm_qw = msg.w;
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
		
		//std::cout << "Joint 0 " << y << "   " << ans_joint0 << " Joint 1 " << x << "   " << ans_joint1;

		pos.data[0]=ans_joint0;
		pos.data[1]=ans_joint1;
	}

	float thresh(float v){
		if (v < 0.001){
			return 0;
		}
		else{
			return v;
		}

	}

	void WristCalc(){

		mat Rforearm0;
		mat Rx90;
		mat Rwrist0;
		mat Rwrist0_0;
		mat RwristForearm;

		Rforearm0 = {{1-2*(pow(foreArm_qy,2)+pow(foreArm_qz,2)), 2*(foreArm_qx*foreArm_qy-foreArm_qw*foreArm_qz), 2*(foreArm_qw*foreArm_qy+foreArm_qx*foreArm_qz)},
             {2*(foreArm_qx*foreArm_qy+foreArm_qw*foreArm_qz), 1-2*(pow(foreArm_qx,2)+pow(foreArm_qz,2)), 2*(foreArm_qy*foreArm_qz-foreArm_qw*foreArm_qx)},
             {2*(foreArm_qx*foreArm_qz-foreArm_qw*foreArm_qy), 2*(foreArm_qw*foreArm_qx+foreArm_qy*foreArm_qz), 1-2*(pow(foreArm_qx,2)+pow(foreArm_qy,2))}};


		Rwrist0 = {{1-2*(pow(wrist_qy,2)+pow(wrist_qz,2)), 2*(wrist_qx*wrist_qy-wrist_qw*wrist_qz), 2*(wrist_qw*wrist_qy+wrist_qx*wrist_qz)},
	             {2*(wrist_qx*wrist_qy+wrist_qw*wrist_qz), 1-2*(pow(wrist_qx,2)+pow(wrist_qz,2)), 2*(wrist_qy*wrist_qz-wrist_qw*wrist_qx)},
	             {2*(wrist_qx*wrist_qz-wrist_qw*wrist_qy), 2*(wrist_qw*wrist_qx+wrist_qy*wrist_qz), 1-2*(pow(wrist_qx,2)+pow(wrist_qy,2))}};

		RwristForearm= Rwrist0*Rforearm0.t();

		mat forearm_diff = foreArm_prev - Rforearm0;
		mat wrist_diff = wrist_prev - Rwrist0;

		// std::cout << "Forearm Diff: " << std::endl;
		// std::cout << Rforearm0 << std::endl;
		// std::cout << "Wrist Rotation: " <<  std::endl;
		// std::cout << Rwrist0 << std::endl;
		// std::cout << "" << std::endl;

		foreArm_prev = Rforearm0;
		wrist_prev = Rwrist0;

		
		vec xF_0=Rforearm0.col(0);
		vec yF_0=Rforearm0.col(1);
		vec zF_0=Rforearm0.col(2);

		vec xW_0=Rwrist0.col(0);
		vec yW_0=Rwrist0.col(1);
		vec zW_0=Rwrist0.col(2);

		float dotzFyW = dot(zF_0,yW_0);
		
		vec czFyW = cross(zF_0, yW_0);
		float mag_cFyW = sqrt(pow(czFyW(0), 2) + pow(czFyW(1), 2) + pow(czFyW(2), 2));
		float dir_zFyW = dot(czFyW, zW_0);

		float wristvalue_z = (atan2(sqrt(1-pow(dotzFyW,2)), dotzFyW)*180/pi);

		wrist_value.data = wristvalue_z;

		//std::cout << "Y: " << Wristy << "  Z: " << wristvalue_z << std::endl;
		
		
		float dotyFzW = dot(yF_0,zW_0);

		vec cyFzW = cross(yF_0, zW_0);
		float mag_cyFzW = sqrt(pow(cyFzW(0), 2) + pow(cyFzW(1), 2) + pow(cyFzW(2), 2));
		
		float wristvalue_y = (atan2(sqrt(1-pow(dotyFzW,2)), dotyFzW)*180/pi);

		pos.data[5] = map(wristvalue_y, jointHomes[6], wristyRange, rangeJoint6);

		//std::cout << "Y: " << wristvalue_y << "  Z: " << wristvalue_z << std::endl;
		
		//wrist_value.y = Wristy;

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