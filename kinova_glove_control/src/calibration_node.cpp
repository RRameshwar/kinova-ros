#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Vector3.h"

#include <sstream>

class IMUCalibrator {
public:
	ros::NodeHandle n;

	ros::Subscriber subMiddle;
	ros::Subscriber subPointer;
	ros::Subscriber subThumb;

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

	bool wristIsCalibrated;
	bool forearmIsCalibrated;
	bool upperIsCalibrated;


	IMUCalibrator(){

        subWrist = n.subscribe("imuWrist", 1000, &IMUCalibrator::WristPosCallback, this);
        subForearm = n.subscribe("imuForearm", 1000, &IMUCalibrator::ForearmPosCallback, this);
        subUpperarm = n.subscribe("imuUpperarm", 1000,  &IMUCalibrator::UpperarmPosCallback, this);
        
  		calibrated = n.advertise<std_msgs::Float32MultiArray>("imuCalibrator", 1000);


        wristIsCalibrated = false;  
        forearmIsCalibrated = false;  
        upperIsCalibrated = false;   

	}

	float calculateAvg(std::list<float> list)
	{
	    double avg = 0;
	    std::list<float>::iterator it;
	    for(it = list.begin(); it != list.end(); it++) avg += *it;
	    avg /= list.size();
		return avg;
	}


	void WristPosCallback(const geometry_msgs::Vector3& msg){
		float x = msg.x;
		float y = msg.y;
		float z = msg.z;

		if (wristX.size() == 60){			
			wristXOffset = calculateAvg(wristX);
			wristYOffset = calculateAvg(wristY);
			wristZOffset = calculateAvg(wristZ);
			std::cout << "WRIST CALIBRATED!" << std::endl;
			wristIsCalibrated = true;
		}

		else{
			wristX.push_back(x);
			wristY.push_back(y);
			wristZ.push_back(z);
			//std::cout << wristX.size() << std::endl;
		}

	}

	void ForearmPosCallback(const geometry_msgs::Vector3& msg){
		float x = msg.x;
		float y = msg.y;
		float z = msg.z;

		if (forearmX.size() == 60){			
			forearmXOffset = calculateAvg(forearmX);
			forearmYOffset = calculateAvg(forearmY);
			forearmZOffset = calculateAvg(forearmZ);
			
			std::cout << "FOREARM CALIBRATED!" << std::endl;
			forearmIsCalibrated = true;
		}

		else{
			forearmX.push_back(x);
			forearmY.push_back(y);
			forearmZ.push_back(z);
			//std::cout << wristX.size() << std::endl;
		}

	}

	void UpperarmPosCallback(const geometry_msgs::Vector3& msg){
		float x = msg.x;
		float y = msg.y;
		float z = msg.z;

		if (upperX.size() == 60){			
			upperXOffset = calculateAvg(upperX);
			upperYOffset = calculateAvg(upperY);
			upperZOffset = calculateAvg(upperZ);
			std::cout << "UPPER X OFFSET: " << upperYOffset << std::endl;
			std::cout << "UPPER ARM CALIBRATED!" << std::endl;
			
			upperIsCalibrated = true;
		}

		else{
			upperX.push_back(x);
			upperY.push_back(y);
			upperZ.push_back(z);
			//std::cout << wristX.size() << std::endl;
		}

	}

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_calibrate");

  IMUCalibrator jc;

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();

    if (/*jc.wristIsCalibrated and */jc.forearmIsCalibrated and jc.upperIsCalibrated){
    	jc.offsets.data.push_back(jc.upperXOffset);
    	jc.offsets.data.push_back(jc.upperYOffset);
    	jc.offsets.data.push_back(jc.upperZOffset);
    	
    	jc.offsets.data.push_back(jc.forearmXOffset);
    	jc.offsets.data.push_back(jc.forearmYOffset);
    	jc.offsets.data.push_back(jc.forearmZOffset);
    	
    	jc.offsets.data.push_back(jc.wristXOffset);
    	jc.offsets.data.push_back(jc.wristYOffset);
    	jc.offsets.data.push_back(jc.wristZOffset);

    	jc.calibrated.publish(jc.offsets);
    	ros::shutdown();
    }

    loop_rate.sleep();
    ++count;
  }

  return 0;
}