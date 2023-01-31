This code allows you to control a robot using a gamepad or sliders in Meshcat using PyDrake. It is originally from TRI and is used in Russ Tedrake's coursework at MIT. The author has given permission to use this code for the purposes of this project.

## How to run:
This code can be run via Bazel run command `bazel run //plannning/teleop:punito` or on an M1 Mac with the `--define NO_DREAL=ON` flag.

## Overview:
  The code consists of several classes:
  1) `MeshcatSliders`: A system that outputs the values from meshcat sliders.
  2) `GamepadCommand`: A system that outputs the values from the gamepad.
  3) `MyController`: A system that calculates the motor torques based on the inputs received from `MeshcatSliders` and `GamepadCommand` systems.

## Usage: 
The code sets up the simulation environment with `MeshcatVisualizer` and creates instances of the above-mentioned classes. The state of the robot, input commands, and motor torques are passed between these classes using ports and are updated in real-time based on user input.
The simulation can be visualized in a web browser by accessing the `Meshcat` server URL printed in the terminal. The user can control the robot by moving the sliders or using the gamepad.

This serves as a great example to get taste of Drake's control and simulation capabilities. It also would serve as a great playground for developing and testing motion planning algorithms. 

Be sure to check out the following courses if you're interested in this!
  - [Underactuated Robotics by Dr. Russ Tedrake](https://underactuated.csail.mit.edu/)
  - [Robotic Manipulation by Dr. Russ Tedrake](https://manipulation.csail.mit.edu/)
  - [More about Mecanum Wheel Kinematics](https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf)
