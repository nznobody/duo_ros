#pragma once
//Add ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <camera_calibration_parsers/parse.h>

//Add the DUO SDK
#include <DUOLib.h>

//Add OpenCV
#include <opencv2/opencv.hpp>

// Standard Includes
#include <stddef.h>

//Hardcoded defaults
#define GAIN		0.0f
#define EXPOSURE	80.0f
#define LEDS		20.0f
#define FPS			30.0f
#define WIDTH		752
#define HEIGHT		480
#define	BINNING		DUO_BIN_NONE

using namespace cv;

namespace duo
{
	class DUOInterface
	{
	public:
		DUOInterface(void);
		~DUOInterface(void);
		
		/*
		* 	@brief
		* 	This outside DUO API function ONLY, can access this DUOStereoDriver class 
		* 	private members since it is listed as a friend to this class. 
		* 	Be careful when using this. 
		*/
		friend void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData);
		
		/*
		*	@brief
		*	Singleton Implementation to allow for only a single instance of this
		*	class. 
		*	-If GetInstance() is called more then once, it will return a pointer
		*	 to a already initialize (non-NULL) pSingleton object.
		*	-If GetInstance() is called for the first time, initialize the pSingleton
		*	 variable to a new DUOStereoDriver object.  
		*
		*	@return
		*	A pointer to the pSingleton private member object. 
		*/
		static DUOInterface&	GetInstance(void)
		{
			if (pSingleton == 0L)
				pSingleton = new DUOInterface();

			return *pSingleton;
		}
		
		/*
		*	@ brief
		*	Check if pSingleton object is not null, and if it is not null, call the 
		*	shutdownDUO() function FIRST, and then delete the pSingleton object.	
		*
		*/
		static void	DestroyInstance(void)
		{
			if (pSingleton != 0L)
			{
				pSingleton->shutdownDUO();

				delete pSingleton;
				pSingleton = NULL;
			}
		}

		bool initializeDUO(void);
		void startDUO(void);
		void shutdownDUO(void);
		
		//Duo parameter functions
		void SetExposure(double val){_exposure = val; SetDUOExposure(_duoInstance, val);}
		void SetGain(double val){_gain = val; SetDUOGain(_duoInstance, val); }
		void SetHFlip(bool val){_flipH = val; SetDUOHFlip(_duoInstance, val); }
		void SetVFlip(bool val){_flipV = val; SetDUOVFlip(_duoInstance, val); }
		void SetCameraSwap(bool val){_swap = val; SetDUOCameraSwap(_duoInstance, val); }
		void SetLedPWM(double val){_leds = val; SetDUOLedPWM(_duoInstance, val); }
	
		static const int TWO_CAMERAS	= 2;
		static const int LEFT_CAM 		= 0;
		static const int RIGHT_CAM		= 1;
		
	protected:
		/*
		*	@ brief Refer to DUO API Docs for these two
		*/
		DUOInstance 		_duoInstance;
		DUOResolutionInfo 	_duoResolutionInfo;
		//Duo parameters
		double	_gain		= GAIN;
		double	_exposure	= EXPOSURE;
		double	_leds		= LEDS;
		bool	_flipH		= false;
		bool	_flipV		= false;
		bool	_swap		= false;
		size_t	_width		= WIDTH;
		size_t	_height		= HEIGHT;
		size_t	_fps		= FPS;
		int		_binning	= BINNING;
		bool	_opencvCalib = true;	//This chooses between openCV calbration or Duo's built in one (only openCV is implemented)
		
		//ROS related
		ros::NodeHandle _camera_nh;
		ros::NodeHandle _single_camera_nh[TWO_CAMERAS];
		boost::shared_ptr<camera_info_manager::CameraInfoManager> _cinfo[TWO_CAMERAS];
		boost::shared_ptr<image_transport::ImageTransport> 	_it;
		image_transport::CameraPublisher 					_imagePub[TWO_CAMERAS];
		
		//General variables
		bool			_duoInitialized = false;
		std::string 	_cameraFrame = "duo_frame";
		static DUOInterface* pSingleton;
		static const std::string CameraNames[TWO_CAMERAS]; // = {"left","right"};
		
		//General member functions
		void fillDUOImages(	sensor_msgs::Image& leftImage, sensor_msgs::Image& rightImage, const PDUOFrame pFrameData);
		void publishImages(	const sensor_msgs::ImagePtr image[TWO_CAMERAS]);
	};
}



