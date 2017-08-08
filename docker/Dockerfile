FROM ros:kinetic

# Change shell to bash
SHELL ["/bin/bash", "-c"]

MAINTAINER Takaki Ueno

# Change environment related to ROS
ENV ROS_PACKAGE_PATH=/root/catkin_ws/src:/opt/ros/kinetic/share
ENV ROS_ETC_DIR=/opt/ros/kinetic/etc/ros
ENV ROS_ROOT=/opt/ros/kinetic/share/ros
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_DISTRO=kinetic
ENV ROSLISP_PACKAGE_DIRECTORIES=/root/catkin_ws/devel/share/common-lisp

# Update repos and install some packages
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install \
    git \
    ros-kinetic-desktop-full \
    ros-kinetic-mavros \
    ros-kinetic-laser-assembler \
    software-properties-common \
    unzip \
    vim \ 
    wget -y
    
# Overwrite bashrc
RUN echo 'source /opt/ros/kinetic/setup.bash' >> /root/.bashrc
RUN echo 'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc

# Source bashrc and setup.bash
RUN source /opt/ros/kinetic/setup.bash
RUN source /root/.bashrc

# Create catkin workspace
RUN mkdir -p /root/catkin_ws/src

# Setup PX4 SITL
RUN usermod -a -G dialout root
RUN wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_gazebo.sh
RUN sed -i 's/sudo//g' ubuntu_sim_ros_gazebo.sh
RUN yes | source ubuntu_sim_ros_gazebo.sh

# Build catkin workspace
RUN cd /root/catkin_ws
RUN catkin_make
