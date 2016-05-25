# duo_ros
A ROS package for the Duo stereo camera from Code Labaoratories.

# Installation
should be able to use git clone combined with catkin_make. I have tried to stick to the standard ROS format even though I am editing this code via Visual Studio with Visual GDB. Note that the camera *.yaml files need to be in cfg/*.yaml reletaive to your present working directory!

So far this will provide basic functionality to ROS. It streams image_raw and camInfo topics. The included launch file uses these with image_proc to generate rectified images and a transform to base_link. This works with RTABMap, but has poor results. The camera settings can be edited with rosrun rqt_reconfigure rqt_reconfigure. Still on the todo list:
 * Implement Duo's Dense3D
 * Abstract camera interface from ROS (This is being done in a seperate repo)
 * Expose more of duo's functionality (LED sequences and so on)
  
Credit for a lot of the implemention style and ideas goes to https://github.com/l0g1x/DUO-Camera-ROS and https://github.com/stereolabs/zed-ros-wrapper who have provided a good base to start from.

Please feel free to help make this the main package for the Duo on ROS
