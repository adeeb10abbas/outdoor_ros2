# Outdoor SLAM and Autonomous Navigation
This is the mono repo for the outdoor SLAM and Autonomous Navigation project at Drexel University. The project is a collaboration between [Drexel Wireless Systems Lab](https://research.coe.drexel.edu/ece/dwsl/) and [Zhou Robotics Lab](https://zhourobotics.github.io/) at [Drexel College of Engineering](http://coe.drexel.edu/).

## Supported Configurations:
  - Ubuntu 22.04 + ROS2 Humble
  - Ubuntu 20.04 + ROS2 Rolling
  - Architecture: x86_64 (amd64), aarch64 (experimental) and Mac M1/Apple Silicon (all `plannning/*` targets)
  - Bazel >= 5.0

## Steps to build and run the projects
- Run the [`ros-humble-install.sh`](https://github.com/Zhourobotics/outdoor_slam_ros2/blob/main/ros-humble-install.sh) for installing all the prereqs needed! 
- To build: `bazel build <target>` and to run `bazel run <target>`; For more please read the [Bazel documentation](https://bazel.build/). For building the whole repo, run `bazel build ...`

- **[Optional/Skip unless you know what you are doing]** If you want to create an overlay ROS2 workspace and use the that. You can use `outdoor.repos` to pull in the required packages via the following steps- 
  ```bash
  mkdir -p outdoor_ws/src
  cd outdoor_ws
  vcs import < `path_to_this_repo`/outdoor.repos src 
  sudo apt-get update
  rosdep init #(if not already done)
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


## Features
- Created an `example`  directory to show how to pull in and build external useful libs like drake-ros, drake, OpenCV and use it in your project for   both `Python` and `C++` targets.
- This package uses Bazel ROS2 rules from [drake-ros](https://github.com/RobotLocomotion/drake-ros)
- List of external packages being pulled in currently and available to use:
  - [drake](https://github.com/RobotLocomotion/drake)
  - [drake-ros](https://github.com/RobotLocomotion/drake-ros)   
  - [Mujoco](https://github.com/deepmind/mujoco/)
  - via [bazel-deps](https://github.com/mjbots/bazel_deps):
    - opencv
    - ffmpeg (and many codecs)
    - gstreamer (and many of its plugins, including X output)
    - eigen
    - boost
    - fmt
  - Python3 (3.10) packages: See [requirements.txt](https://github.com/zhourobotics/outdoor_slam_ros2/blob/main/requirements.txt)

## Maintainers:
 - Adeeb Abbas: [@adeeb10abbas](https://github.com/adeeb10abbas)
