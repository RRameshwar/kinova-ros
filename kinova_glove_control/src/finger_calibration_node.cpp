#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/Vector3.h"

#include <sstream>

class FingerCalibrator {
public:
	ros::NodeHandle n;

	ros::Subscriber subFingers;

	ros::Publisher calibrated;

	std_msgs::Int16MultiArray allVals;

	std::list<float> thumbAngles;
	std::list<float> pointerAngles;
	std::list<float> middleAngles;

	int mode;
	bool fingersOpen;
	bool fingersClosed;


	FingerCalibrator(){
		subFingers = n.subscribe("fingersPos", 1000, &FingerCalibrator::FingersPosCallback, this);
        
  		calibrated = n.advertise<std_msgs::Int16MultiArray>("fingerCalibrator", 1000);

  		mode = 0;
        fingersOpen = false;
        fingersClosed = false;
	}

	float calculateAvg(std::list<float> list)
	{
	    double avg = 0;
	    std::list<float>::iterator it;
	    for(it = list.begin(); it != list.end(); it++) avg += *it;
	    avg /= list.size();
		return avg;
	}


	void FingersPosCallback(const std_msgs::Int16MultiArray& msg){
	  int thumb =  msg.data[0];
	  int pointer = msg.data[1];
	  int middle = msg.data[2];

	  if (mode == 0){
		  if(middleAngles.size()<60){
		  	thumbAngles.push_back(thumb);
		  	pointerAngles.push_back(pointer);
		  	middleAngles.push_back(middle);
		  	std::cout << "Calibrating Open Position" << std::endl;
		  }
		  else{
		  	allVals.data.push_back(calculateAvg(thumbAngles));		  	
		  	allVals.data.push_back(calculateAvg(pointerAngles));
		  	allVals.data.push_back(calculateAvg(middleAngles));
		  	allVals.data.push_back(calculateAvg(middleAngles));
		  	allVals.data.push_back(calculateAvg(middleAngles));

		  	thumbAngles.clear();
		  	middleAngles.clear();
		  	pointerAngles.clear();
		  	fingersOpen = true;
		  	mode = 1;
		  	std::cout << "CLOSE FINGERS" << std::endl;
		  	sleep(1);
		  }
	  }
	  else{
	  	if(middleAngles.size()<60){
		  	thumbAngles.push_back(thumb);
		  	pointerAngles.push_back(pointer);
		  	middleAngles.push_back(middle);
		  	std::cout << "Calibrating Closed Position" << std::endl;
		  }
		  else{
		  	allVals.data.push_back(calculateAvg(thumbAngles));		  	
		  	allVals.data.push_back(calculateAvg(pointerAngles));
		  	allVals.data.push_back(calculateAvg(middleAngles));
		  	allVals.data.push_back(calculateAvg(middleAngles));
		  	allVals.data.push_back(calculateAvg(middleAngles));
		  	fingersClosed = true;
		  }
	  }
	}

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "finger_calibrate");

  FingerCalibrator jc;

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();

    if (jc.fingersClosed and jc.fingersOpen){
    	jc.calibrated.publish(jc.allVals);
    	std::cout << "CALIBRATED FINGERS" << std::endl;
    	
    	for (int j = 0; j < 10; j++){
    		std::cout << jc.allVals.data[j] << std::endl;
    	}
    	std::cout << "PRINTED";
    	ros::shutdown();
    }

    loop_rate.sleep();
    ++count;
  }

  return 0;
}