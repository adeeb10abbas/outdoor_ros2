# Outdoor SLAM and Autonomous Navigation
This is the mono repo for the outdoor SLAM and autonomous navigation project at Drexel University. The project is a collaboration between Drexel Wireless Systems Lab and Zhou Robotics Lab at Drexel College of Engineering.

## Building and running the mono Repo 
- Need to have Bazel installed (>=5.0)
- `bazel build ...`
- Bazel ROS2 rules from [drake-ros](https://github.com/RobotLocomotion/drake-ros)
- Created an `sample_cpp` sub directory to show how to pull in and build external useful libs like OpenCV and use it. 


## Creating new functionalities 
- Try to mimic any one of the examples provided (BUILD, environ.bzl etc.) as needed. 

TODO: @adeeb10abbas - Add more relevant info here.
