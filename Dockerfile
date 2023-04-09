# Author: Adeeb Abbas
# Decription: Dockerfile for building a Docker image for ROS 2 Humble w/ Jammy. Allows us to build our ROS2 repos easily in the lab. 
# Steps to use it (assuming you have Docker installed):
# 1. Clone this repo
# 2. cd into the repo
# 3. Run the following commands - 
#    docker build -t ros-humble-slam .
#    docker run -it --rm ros-humble-slam
# Once you're in the container, you can run the following commands to build the Outdoor ROS repos -
# clone the outdoor_ros2_slam repo. This repo contains the ROS2 packages for the SLAM project (#TODO: Automate this step/make it a part of the Dockerfile)
# bazel build //... --define NO_DREAL=ON -j 1 
# Limiting the jobs just because you might run out of memory as the build is memory intensive, and it's painful to 
# to restart the whole thing again. Feel free to change it. Make sure your docker settings allow upto 8gb ram. 

# Use the official Ubuntu 22.04 as the base image
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Update and upgrade the system
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y tzdata

# Install necessary dependencies for the script
RUN apt-get install -y wget unzip curl software-properties-common lsb-release python3-pip

# Install npm
RUN apt-get install -y npm

# Install Bazelisk using npm
RUN npm install -g @bazel/bazelisk

# Run Bazelisk to install Bazel
RUN bazelisk

# Run the commands from the script (slightly modified ros-humble-installer.sh) directly
RUN wget -q -O /tmp/drake-setup.zip https://github.com/RobotLocomotion/drake/archive/refs/heads/master.zip && \
    unzip -q /tmp/drake-setup.zip -d /tmp && \
    yes | bash /tmp/drake-master/setup/ubuntu/install_prereqs.sh && \
    rm -rf /tmp/drake-setup.zip /tmp/drake-master && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get install -y curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get install -y curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y ros-humble-desktop && \
    apt-get install -y ros-dev-tools && \
    apt-get install libglfw3 && \
    apt-get install libglfw3-dev && \
    # it will yell at you because it's root and what not.. just ignore it
    rosdep init && \ 
    rosdep update && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    apt-get update && \
    apt-get install -y python3-colcon-common-extensions && \
    apt-get install -y ros-humble-rmw-cyclonedds-cpp && \
    pip3 install toposort && \
    apt-get install -y $(apt-cache search gflags | awk '{print $1}') && \
    apt-get install -y libsuitesparse-dev && \
    apt-get install -y ros-humble-vision-opencv

# Set the entrypoint to source ROS setup.bash and run a bash shell
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec /bin/bash -i"]
