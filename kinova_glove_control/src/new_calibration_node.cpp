


#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Vector3.h"

#include <sstream>



class SensorCalibrator {
public:
	ros::NodeHandle n;

	ros::Subscriber subFingers;
	ros::Subscriber sub;

	ros::Subscriber subWrist;
	ros::Subscriber subForearm;
	ros::Subscriber subUpperarm;

	ros::Publisher calibrated;

	std_msgs::Float32MultiArray offsets;

	std::list<float> wristX;
	std::list<float> wristY;
	std::list<float> wristZ;

	std::list<float> forearmX;	
	std::list<float> forearmY;
	std::list<float> forearmZ;
	
	std::list<float> upperX;
	std::list<float> upperY;
	std::list<float> upperZ;

	float wristXOffset;
	float wristYOffset;
	float wristZOffset;

	float forearmXOffset;
	float forearmYOffset;
	float forearmZOffset;

	float upperXOffset;
	float upperYOffset;
	float upperZOffset;

	int calibrationState;

	SensorCalibrator(){ 
        
  		calibrated = n.advertise<std_msgs::Float32MultiArray>("imuCalibrator", 1000);

  		calibrationState = 0;

	}

	void UpperarmPosCB(const geometry_msgs::Vector3& msg){
		
		for (int i = 0; i < 10; i++){
			std::cout << "CALIBRATING UPPER ARM" << std::endl;
		}
		calibrationState = 1;
	}

	void ForearmPosCB(const geometry_msgs::Vector3& msg){
		
		for (int i = 0; i < 10; i++){
			std::cout << "CALIBRATING FOREARM" << std::endl;
		}
		calibrationState = 2;
	}

	void WristPosCB(const geometry_msgs::Vector3& msg){
		
		for (int i = 0; i < 10; i++){
			std::cout << "CALIBRATING WRIST" << std::endl;
		}
		calibrationState = 3;
	}

	void FingersPosCB(const std_msgs::Float32MultiArray& msg){
		
		for (int i = 0; i < 10; i++){
			std::cout << "CALIBRATING FINGERS" << std::endl;
		}
		calibrationState = 4;
	}

	void RunCalibration(){
		if(calibrationState == 0){
			sub = n.subscribe("imuUpperarm", 1000,  &SensorCalibrator::UpperarmPosCB, this);
		}

		if(calibrationState == 1){
			std::cout << "CALIBRATED UPPER ARM" << std::endl;
			//sub.shutdown();
			sub = n.subscribe("imuForearm", 1000, &SensorCalibrator::ForearmPosCB, this);
		}

		else if(calibrationState == 2){
			std::cout << "CALIBRATED FOREARM" << std::endl;
			//sub.shutdown();
			sub = n.subscribe("imuWrist", 1000, &SensorCalibrator::WristPosCB, this);
		}

		else if(calibrationState == 3){
			std::cout << "CALIBRATED WRIST" << std::endl;
			//sub.shutdown();
			sub = n.subscribe("fingersPos", 1000, &SensorCalibrator::FingersPosCB, this);
		}

		else if(calibrationState == 4){
			std::cout << "CALIBRATED FINGERS" << std::endl;
			//sub.shutdown();
		}
		else {
			std::cout << calibrationState << std::endl;
		}

	}
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_calibrate");

  SensorCalibrator sc;

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();

    sc.RunCalibration();

    if (sc.calibrationState == 4){
    	std::cout << "CALIBRATED ALL" << std::endl;
    	ros::shutdown();
    }

    loop_rate.sleep();
    ++count;
  }

  return 0;
}