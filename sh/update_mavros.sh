#!/bin/bash
#
# Script to update mavros following instruction in https://dev.px4.io/en/ros/mavros_installation.html
# Usage: source update_mavros.sh

# Remove existing mavros
cd ~/catkin_ws/src
rm -rf mavros

cd ~/catkin_ws
wstool init src

rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavos.rosinstall
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall

wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src

if ! rosdep install --from-paths src --ignore-src --rosdistro kinetic -y; then
    # (Use echo to trim leading/trailing whitespaces from the unsupported OS name
    unsupported_os=$(echo $(rosdep db 2>&1| grep Unsupported | awk -F: '{print $2}'))
    rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --os ubuntu:xenial
fi

catkin build
