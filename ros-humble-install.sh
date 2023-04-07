#!/bin/bash

# Author: Adeeb Abbas
# Date: 2023-03-19
# Script: ros-humble-install.sh

# Description: This script installs ROS 2 Humble + (other ROS stuff like Colcon, rosdep) and 
# Drake prerequisites on Ubuntu 22.04, providing a simple, and straightforward setup for running 
# both drake-ros and this repo. 

# Instructions:
# 1. Ensure the script has executable permissions: chmod +x ros-humble-install.sh
# 2. Execute the script with sudo: sudo ./ros-humble-install.sh

set -e

skip_rosdep=false
# Process command-line arguments
while [ "$#" -gt 0 ]; do
    case "$1" in
        --skip-rosdep)
            skip_rosdep=true
            shift
            ;;
        *)
            echo "Unknown option: $1" >&2
            exit 1
            ;;
    esac
done

function main {
    install_drake_prerequisites
    add_universe_repository
    update_install_curl
    get_ros_key
    update_upgrade_system
    install_ros
    configure_bash
    update_system
    install_colcon
    install_cyclonedds
    install_toposort
    install_vision_opencv
    if [ "$skip_rosdep" = "false" ]; then
        install_rosdep
    fi

    install_gflags
    install_suiteparse
}

install_drake_prerequisites() {
  echo "Installing Drake prerequisites..."

  temp_dir=$(mktemp -d)
  trap 'rm -rf -- "$temp_dir"' EXIT

  wget -q -O "${temp_dir}/drake-setup.zip" https://github.com/RobotLocomotion/drake/archive/refs/heads/master.zip

  unzip -q "${temp_dir}/drake-setup.zip" -d "${temp_dir}"
  drake_setup_dir="${temp_dir}/drake-master/setup/ubuntu"

  chmod +x "${drake_setup_dir}/install_prereqs.sh"

  # Run the script without changing the working directory
  sudo "${drake_setup_dir}/install_prereqs.sh"
}

function add_universe_repository {
    echo "Adding universe repository..."
    sudo add-apt-repository universe
}

function update_install_curl {
    echo "Updating and installing curl..."
    sudo apt update && sudo apt install -y curl
}

function get_ros_key {
    echo "Getting ROS key..."
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
}

function update_upgrade_system {
    echo "Updating and upgrading system..."
    sudo apt update && sudo apt upgrade -y
}

function install_ros {
    echo "Installing ROS Humble Desktop..."
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    # Install the desktop user entry point.
    sudo apt install ros-humble-desktop
    # Install dev tools.
    sudo apt install ros-dev-tools
    # Update dependencies index.
    rosdep update
}

function configure_bash {
    echo "Configuring bash..."
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
}

function add_ros2_repository {
    echo "Adding ROS2 repository..."
    sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
}

function add_ros_key {
    echo "Adding ROS key..."
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
}

function update_system {
    echo "Updating system..."
    sudo apt update
}

function install_colcon {
    echo "Installing colcon..."
    sudo apt install -y python3-colcon-common-extensions
}

function install_cyclonedds {
    echo "Installing CycloneDDS..."
    sudo apt-get install -y ros-humble-rmw-cyclonedds-cpp
}

function install_toposort {
    echo "Installing toposort..."
    pip3 install toposort
}

function install_rosdep {
    echo "Installing rosdep..."
    sudo apt install -y python3-rosdep2
    rosdep init
    rosdep update
}

function install_gflags {
    echo "Installing gflags..."
    sudo apt-get install -y $(apt-cache search gflags | awk '{print $1}')
}

function install_suiteparse {
    # Needed for building opencv
    echo "Installing SuiteSparse..."
    sudo apt-get install -y libsuitesparse-dev
}

function install_vision_opencv {
	sudo apt-get install -y ros-humble-vision-opencv
}

main
