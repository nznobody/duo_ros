//Include ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <duo_ros/DuoConfig.h>

#include <iostream>
#include <signal.h>

#include "DUOInterface.h"

using namespace std;

void sigIntHandler(int sig)
{
	ROS_DEBUG("--> SIGINT Handler called <--");

	    //Destory our pSingleton Instance that we created.
	duo::DUOInterface::DestroyInstance();

	    // Tell the ros node that we want to shutdown, so we receive a 
	    // clean exit
	ros::shutdown();
}

void callback(duo_ros::DuoConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %f %f %f",config.gain, config.exposure, config.leds);
	duo::DUOInterface& _duo = duo::DUOInterface::GetInstance();
	_duo.SetExposure(config.exposure);
	_duo.SetGain(config.gain);
	_duo.SetLedPWM(config.leds);
	_duo.SetCameraSwap(config.cameraSwap);
	_duo.SetHFlip(config.flipH);
	_duo.SetVFlip(config.flipV);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "duo_ros_node");
	duo::DUOInterface& _duo = duo::DUOInterface::GetInstance();
	_duo.initializeDUO();
	_duo.startDUO();
	
	//Register and start dynamic reconfigure node
	dynamic_reconfigure::Server<duo_ros::DuoConfig> server;
	dynamic_reconfigure::Server<duo_ros::DuoConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);
	
	signal(SIGINT, sigIntHandler);
	
	ROS_INFO("Spinning node");
	ros::spin();
	return 0;

}