# ROS 2 Jazzy + Gazebo Harmonic image for the `gazebo` CI job (E2E-07), amd64.
FROM ros:jazzy-ros-base
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers ros-jazzy-xacro ros-jazzy-robot-state-publisher \
      ros-jazzy-urdfdom liburdfdom-tools ros-jazzy-controller-manager \
      cmake ninja-build g++ libeigen3-dev libgtest-dev libgmock-dev libtinyxml2-dev \
      libyaml-cpp-dev python3-yaml python3-numpy python3-pandas python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
SHELL ["/bin/bash", "-c"]
