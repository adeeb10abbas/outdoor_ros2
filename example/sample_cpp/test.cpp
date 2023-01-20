#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake_ros_core/drake_ros.h>
#include <drake_ros_core/ros_interface_system.h>
#include <drake_ros_viz/rviz_visualizer.h>
#include <opencv2/core.hpp>

/*Mujoco test includes*/
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mjxmacro.h>
#include <mujoco/mjui.h>
#include <mujoco/mujoco.h>

int main() {
  /* This is a test file which serves as a basic example of how to use the drake_ros_* packages with the
  BAZEL build system. This file is not meant to be run, but rather to be used as a reference for how
  to use the drake_ros_* packages. */

  cv::Mat _m; // This is just to show that OpenCV is available and functioning properly.
  
  // MuJoCo data structures
  mjModel* m __attribute__((unused));                  // MuJoCo model
  mjData* d __attribute__((unused));                   // MuJoCo data
  mjvCamera cam __attribute__((unused));                      // abstract camera
  mjvOption opt __attribute__((unused));                      // visualization options
  mjvScene scn __attribute__((unused));                       // abstract scene
  mjrContext con __attribute__((unused));                     // custom GPU context
  mjvPerturb pert __attribute__((unused));                    // perturbation
  mjuiState uistate __attribute__((unused));                  // user interface state

  return 0;
}