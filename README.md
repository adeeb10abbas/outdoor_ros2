# Outdoor SLAM and autonomous navigation
## Building and running the mono Repo 
- Need to have Bazel installed (>=5.0)
- `bazel build ...`
- `example/` - There's some simple chatter, actions and zero copy chatter examples taken from [rules_ros2](https://github.com/mvukov/rules_ros2) for reference
- Bazel ROS2 rules from [drake-ros](https://github.com/RobotLocomotion/drake-ros)
- Created an `image_tools` sub directory to show how to pull in and build external useful libs like OpenCV and use it. 

TODO: @adeeb10abbas - Add more details

# Running examples 
- `bazel run //example/chatter:talker`
- `bazel run //example/chatter:listener`

# Creating new functionalities 
- Try to mimic any one of the examples provided (BUILD, environ.bzl etc.) as needed. 
