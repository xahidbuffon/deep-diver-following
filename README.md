This is a container for an autonomous diver-following project. Deep object detection models are used for diver (and other objects such as ROV) detection. A simplified version of that is utilized for autonomous tracking (and following) of a (single) diver by an underwater robot. The ROS version, tested on Aqua-8 robot, is provided in the diver_following_cnn folder.

- Paper link:  https://ieeexplore.ieee.org/document/8543168
- Dataset collection information:  https://onlinelibrary.wiley.com/doi/full/10.1002/rob.21837 
- A trained frozen model: provided in model_data folder

## Testing the detector
For testing individual images, run the [test_detector.py](test_detector.py) file. Change the image directory (im_dir) to test other images of interest.

| Multiple divers | Divers and ROVs | 
|:--------------------|:----------------|
| ![det-7](/test_data/res/7.jpg)     | ![det-1](/test_data/res/1.jpg) |


## Testing the diver-tracker
For testing on a diver-tracking video or sequences of images, run the [test_diver_tracker.py](test_diver_tracker.py) file. A couple of videos and image sequences are provided in the test_data folder. Change the argument values to test on other files.


[![Demo Doccou alpha](/test_data/res/7.jpg)](https://www.youtube.com/embed/bx6R8uR7MC4?autoplay=1)






## ROS version
- The diver_following_cnn folder contain the ROS-package version 
- This version is currently running on the Aqua MinneBot robot (more details: http://irvlab.cs.umn.edu)
- Feel free to cite the paper you find anything useful:  https://ieeexplore.ieee.org/document/8543168
```
@article{islam2018towards,
  title={Towards a Generic Diver-Following Algorithm: Balancing Robustness and Efficiency in Deep Visual Detection},
  author={Islam, Md Jahidul and Fulton, Michael and Sattar, Junaed},
  journal={IEEE Robotics and Automation Letters (RA-L)},
  volume = {4},
  number = {1},
  year={2018},
  publisher={IEEE}
}
```
