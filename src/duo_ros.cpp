//Include ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <duo_ros/DuoConfig.h>

#include <iostream>
#include <signal.h>

#include "DUOInterface.h"

using namespace std;

void sigIntHandler(int sig) {
	ROS_DEBUG("--> SIGINT Handler called <--");

	    //Destory our pSingleton Instance that we created.
	duo::DUOInterface::DestroyInstance();

	    // Tell the ros node that we want to shutdown, so we receive a 
	    // clean exit
	ros::shutdown();
}

void callback(duo_ros::DuoConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %f %f %f", config.gain, config.exposure, config.leds);
	duo::DUOInterface& _duo = duo::DUOInterface::GetInstance();
	_duo.SetExposure(config.exposure);
	_duo.SetGain(config.gain);
	_duo.SetLedPWM(config.leds);
	_duo.SetCameraSwap(config.cameraSwap);
	_duo.SetHFlip(config.flipH);
	_duo.SetVFlip(config.flipV);
}

void publishCamInfo(sensor_msgs::CameraInfoPtr cam_info_msg, ros::Publisher pub_cam_info) {
	static int seq = 0;
	cam_info_msg->header.seq = seq;
	pub_cam_info.publish(cam_info_msg);
	seq++;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "duo_ros_node");
	duo::DUOInterface& _duo = duo::DUOInterface::GetInstance();
	if(_duo.initializeDUO())
	{
		_duo.startDUO();
	}
	else
		return -1;
	
	
	//Start moving ROS parts out of DUOInterface
//	std::string img_topic = "image_raw";
//	
//	string left_topic = "left/" + img_topic;
//	string left_cam_info_topic = "left/camera_info";
//	string left_frame_id = "/duo_left_optical_frame";
//
//	string right_topic = "right/" + img_topic;
//	string right_cam_info_topic = "right/camera_info";
//	string right_frame_id = "/duo_right_optical_frame";
//	// Camera info publishers
//	ros::NodeHandle nh;
//	ros::Publisher pub_rgb_cam_info = nh.advertise<sensor_msgs::CameraInfo>(left_cam_info_topic, 1); //rgb
//	ROS_INFO_STREAM("Advertized on topic " << left_cam_info_topic);
//	ros::Publisher pub_left_cam_info = nh.advertise<sensor_msgs::CameraInfo>(right_cam_info_topic, 1); //left
//	ROS_INFO_STREAM("Advertized on topic " << right_cam_info_topic);
//	
//	sensor_msgs::CameraInfoPtr left_cam_info_msg =  _duo.GetCamInfoL()->getCameraInfo() );
//	sensor_msgs::CameraInfoPtr right_cam_info_msg = ( _duo.GetCamInfoL()->getCameraInfo() );
	
	//Register and start dynamic reconfigure node
	dynamic_reconfigure::Server<duo_ros::DuoConfig> server;
	dynamic_reconfigure::Server<duo_ros::DuoConfig>::CallbackType f;
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);
	
	signal(SIGINT, sigIntHandler);
	ros::Rate loop_rate(20);
	
	try {
        // Main loop
		while (ros::ok()) {
			//ROS_INFO("Spinning node");
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	catch (...) {
		ROS_ERROR("Unknown error.");
		return 1;
	}
	return 0;
}