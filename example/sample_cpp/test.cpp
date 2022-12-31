#include "opencv2/core.hpp"
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/leaf_system.h>

#include <drake_ros_core/drake_ros.h>
#include <drake_ros_core/ros_interface_system.h>

#include <drake_ros_viz/rviz_visualizer.h>

int main() {
  /* This is a test file which serves as a basic example of how to use the drake_ros_* packages with the
  BAZEL build system. This file is not meant to be run, but rather to be used as a reference for how
  to use the drake_ros_* packages. */

  cv::Mat m; // This is just to show that OpenCV is available and functioning properly.
  
  return 0;
}