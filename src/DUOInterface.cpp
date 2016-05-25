#include "DUOInterface.h"

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

using namespace cv;

namespace duo
{
	const std::string DUOInterface::CameraNames[TWO_CAMERAS] = { "left", "right" };
	
	DUOInterface* DUOInterface::pSingleton(0L);
	
	DUOInterface::DUOInterface(void)
		: _camera_nh("duo3d_camera"),
		_it(new image_transport::ImageTransport(_camera_nh))
	{
		for (size_t i = 0; i < TWO_CAMERAS; i++)
		{
			_single_camera_nh[i]	= ros::NodeHandle(_camera_nh, CameraNames[i]);
			_cinfo[i] = boost::shared_ptr<camera_info_manager::CameraInfoManager> (new camera_info_manager::CameraInfoManager(_single_camera_nh[i]));
			_imagePub[i] 			= _it->advertiseCamera(CameraNames[i] + "/image_raw", 1);
		}
	}


	DUOInterface::~DUOInterface(void)
	{
		CloseDUO(_duoInstance);
	}
	
	void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData)
	{

		// Using singleton to access DUOInterface 
		DUOInterface& 		duoDriver 	= DUOInterface::GetInstance(); 	

		sensor_msgs::ImagePtr 	image[DUOInterface::TWO_CAMERAS];	
		for (int i = 0; i < DUOInterface::TWO_CAMERAS; i++)
		{
			image[i] = sensor_msgs::ImagePtr(new sensor_msgs::Image);    	
		}

		duoDriver.fillDUOImages(*image[DUOInterface::LEFT_CAM], *image[DUOInterface::RIGHT_CAM], pFrameData);
		
		duoDriver.publishImages(image);
	}

	bool DUOInterface::initializeDUO()
	{
		// Implement libCheck() later to tell user they need to update their DUO SDK
		ROS_DEBUG("DUOLib Version: %s", GetLibVersion());

		/*
		* @brief
		* Select 752x480 resolution with no binning capturing at 30FPS
		* These values (width, height, FPS) should be ROS Params
		*/
		if (EnumerateResolutions(&_duoResolutionInfo, 1, _width, _height, _binning, _fps))
		{
			ROS_INFO("Resolution Parameters Check: PASSED");
			if (OpenDUO(&_duoInstance))
			{
				char	_duoDeviceName[252];
				char	_duoDeviceSerialNumber[252];
				char	_duoDeviceFirmwareVersion[252];
				char	_duoDeviceFirmwareBuild[252];
				GetDUODeviceName(_duoInstance, _duoDeviceName);
				GetDUOSerialNumber(_duoInstance, _duoDeviceSerialNumber);
				GetDUOFirmwareVersion(_duoInstance, _duoDeviceFirmwareVersion);
				GetDUOFirmwareBuild(_duoInstance, _duoDeviceFirmwareBuild);
				SetDUOResolutionInfo(_duoInstance, _duoResolutionInfo);

				SetDUOExposure(_duoInstance, _exposure);
				SetDUOGain(_duoInstance, _gain);
				SetDUOLedPWM(_duoInstance, _leds);
				SetDUOCameraSwap(_duoInstance, false);
				
				//Note: We are NOT using unidistort or camera intrinsics due to openCV Calibration being used.
				if (_opencvCalib)
				{
					SetDUOUndistort(_duoInstance, false);
					//Load openCV calib files
					std::string	camNameL;
					sensor_msgs::CameraInfo	camInfoL;
					//Left camera
					camera_calibration_parsers::readCalibration("cfg/duo_left.yaml", camNameL, camInfoL);
					if (camNameL == "duo_left")
					{
						_cinfo[LEFT_CAM]->setCameraName(camNameL);
						_cinfo[LEFT_CAM]->setCameraInfo(camInfoL);
					}
					else
						return false;
					//Right camera
					std::string	camNameR;
					sensor_msgs::CameraInfo	camInfoR;
					camera_calibration_parsers::readCalibration("cfg/duo_right.yaml", camNameR, camInfoR);
					if (camNameR == "duo_right")
					{
						_cinfo[RIGHT_CAM]->setCameraName(camNameR);
						_cinfo[RIGHT_CAM]->setCameraInfo(camInfoR);
					}
					else
						return false;
				}
				else
				{
					//Use Duo Calib //TODO
					SetDUOUndistort(_duoInstance, true);
				}
				_duoInitialized = true;
			}
			else
			{
				ROS_ERROR("Cannot Open DUO. Please check connection!");
				_duoInitialized = false;
			}
		}
		else
		{
			ROS_ERROR("Resolution Parameters Check: FAILED");
			_duoInitialized = false;
		}
		return _duoInitialized;
	}
	
	void DUOInterface::startDUO()
	{
		if (_duoInitialized)
		{
		// If we could successfully open the DUO, then lets start it to finish
		// the initialization 
			ROS_INFO("Starting DUO...");
			StartDUO(_duoInstance, DUOCallback, NULL);
			ROS_INFO("DUO Started.");
		}	
	}

	void DUOInterface::shutdownDUO()
	{
		ROS_WARN("Shutting down DUO Camera.");
		StopDUO(_duoInstance);
	}
	
	void DUOInterface::fillDUOImages(sensor_msgs::Image& leftImage, sensor_msgs::Image& rightImage, const PDUOFrame pFrameData)
	{
		leftImage.header.stamp 		= ros::Time(double(pFrameData->timeStamp) * (1.e-4));
		leftImage.header.frame_id 	= _cameraFrame;
		rightImage.header.stamp 	= leftImage.header.stamp;
		rightImage.header.frame_id 	= _cameraFrame;
		
		sensor_msgs::fillImage(	leftImage, 			// image reference
			sensor_msgs::image_encodings::MONO8, 	// type of encoding
			pFrameData->height, 					// columns in pixels 
			pFrameData->width,						// rows in pixels
			pFrameData->width,						// step size 
			pFrameData->leftData);					// left camera data pointer

		sensor_msgs::fillImage( rightImage,
			sensor_msgs::image_encodings::MONO8,
			pFrameData->height,
			pFrameData->width,
			pFrameData->width,
			pFrameData->rightData);
	}
	
	void DUOInterface::publishImages(const sensor_msgs::ImagePtr image[TWO_CAMERAS])
	{
		for (int i = 0; i < TWO_CAMERAS; i++)
		{
		    // Get current CameraInfo data and populate ImagePtr Array
			sensor_msgs::CameraInfoPtr	ci(new sensor_msgs::CameraInfo(_cinfo[i]->getCameraInfo()));
			// If camera info and image width and height dont match
			// then set calibration_matches to false so we then 
			// know if we should reset the camera info or not
			ci->header.frame_id = image[i]->header.frame_id;
			ci->header.stamp 	= image[i]->header.stamp;

			_imagePub[i].publish(image[i], ci);
		}

		sensor_msgs::clearImage(*image[0]);
		sensor_msgs::clearImage(*image[1]);
	}
}







