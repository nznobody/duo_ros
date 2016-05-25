# duo_ros
A ROS package for the Duo stereo camera from Code Labaoratories.

# Installation
should be able to use git clone combined with catkin_make. I have tried to stick to the standard ROS format even though I am editing this code via Visual Studio with Visual GDB. Note that the camera *.yaml files need to be in cfg/*.yaml reletaive to your present working directory!

So far basic image_raw streaming has been implemented. Still needs:
  * CameraInfo
  * IMU
  * Change Resolution
  * Use Duo Calib (currently uses openCV due to an issue with the fisheye lenses of the MLX)
  * soooooo much more
  
Credit for a lot of the implemention style and ideas goes to https://github.com/l0g1x/DUO-Camera-ROS and https://github.com/stereolabs/zed-ros-wrapper who have provided a good base to start from.

Please feel free to help make this the main package for the Duo on ROS
