*** This folder the ROS-package version that is currently (Dec 2018) running on the Aqua MinneBot robot. 
*** For more details visit our lab website: irvlab.cs.umn.edu  

Project for CNN-based diver following:

>> This project contain code for diver detection and publishing the target bounding box
  > the target bounding box is used by yaw_pitch_controller for generating actual commands 
    for following the diver (see target detection project, bbox_yaw_pitch_controller.py)    

>> relevant papers: 
    >> https://ieeexplore.ieee.org/document/8543168
    >> https://onlinelibrary.wiley.com/doi/full/10.1002/rob.21837

>> how to run:
    >> the whole diver following module:
	> launch: roslaunch diver_following_cnn diver_follow_xahid.launch

    >> only the diver detection module:
	> launch: roslaunch diver_following_cnn diver_detection_xahid.launch
	> rosrun: rosrun diver_following_cnn follow_go.py

