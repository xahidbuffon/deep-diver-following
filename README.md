This is a container of modules for an autonomous diver-following project. 
  

## Testing the detector
For testing individual images, run the test_detector.py file

| CMU's Original Model</br> on Macbook Pro 15" | Mobilenet Variant </br>on Macbook Pro 15" | Mobilenet Variant</br>on Jetson TX2 |
|:---------|:--------------------|:----------------|
| ![cmu-model](/test_data/res/7.jpg)     | ![mb-model-macbook](/test_data/res/1.jpg) | ![mb-model-tx2](/test_data/res/0.jpg) |
| **~0.6 FPS** | **~4.2 FPS** @ 368x368 | **~10 FPS** @ 368x368 |


## Testing the diver-tracker
Run the test_diver_tracker.py 


## ROS version
- The diver_following_cnn folder contain the ROS-package version 
- This version is currently running on the Aqua MinneBot robot (more details: http://irvlab.cs.umn.edu)
- Paper:  https://ieeexplore.ieee.org/document/8543168

