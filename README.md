# Outdoor SLAM and Autonomous Navigation
This is the mono repo for the outdoor SLAM and autonomous navigation project at Drexel University. The project is a collaboration between Drexel Wireless Systems Lab and Zhou Robotics Lab at Drexel College of Engineering.

## Steps to build and run the project

- Need to have Bazel (>=5.0) and ROS2 installed ([the debian way](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html))
- Create an overlay ROS2 workspace and use the given `outdoor.repos` to pull in the required packages via the following steps- 
  ```bash
  mkdir -p outdoor_ws/src
  cd outdoor_ws
  vcs import < <path_to_this_repo>/outdoor.repos src 
  sudo apt-get update
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --symlink-install
  ```
- Bind a local ROS 2 workspace underlay in your WORKSPACE (given in the root of this project):
  ```starlark
  ros2_local_repository(
      name = "ros2",
      workspace = ["/opt/ros/<distro>", "<path_to_your_outdoor_ws>/install"],
  )
  ```
  NOTE: Multiple ROS workspaces can be added and just need to be added as a string in a `workspace` list as shown. 
- To build: `bazel build <target>` and to run `bazel run <target>`; For more please read the [Bazel documentation](https://bazel.build/). 
- Bazel ROS2 rules from [drake-ros](https://github.com/RobotLocomotion/drake-ros)
- Created an `sample_cpp` sub directory to show how to pull in and build external useful libs like drake-ros, drake, OpenCV and use it. 