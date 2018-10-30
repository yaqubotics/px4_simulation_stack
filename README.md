# px4_simulation_stack
Package for PX4 SITL Gazebo simulation.  

Documentation is available on [github pages](https://uenota.github.io/px4_simulation_stack/).

## Compatible version of PX4 Firmware
The master branch may not be compatible with the latest master or release of [PX4 Firmware](https://github.com/PX4/Firmware).  
Please see [releases](https://github.com/uenota/px4_simulation_stack/releases) page for compatible commit.

## File List
You can see file list from https://uenota.github.io/px4_simulation_stack/files.html.

## Docker
### Build Image
```bash
cd ~/catkin_ws/src/px4_simulation_stack/docker
docker build -t uenota/px4sim:1.0 .
```
  
### Run Container
See http://wiki.ros.org/docker/Tutorials/GUI.  
It is neccessary to build PX4 files for the first time.
```bash
make posix_sitl_defalut gazebo
```
  
## Without Docker
### PX4 Installation
See https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html.  
  
### Setting Up PX4 SITL
See https://dev.px4.io/en/simulation/ros_interface.html.  
  
It is convenient to write settings in .bashrc and .profile.  
  
In Firmware directory,
```bash
echo "source $(pwd)/Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$(pwd)" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=\$GAZEBO_PLUGIN_PATH:$(pwd)/build/posix_sitl_default/build_gazebo" >> ~/.bashrc
```
  
### Exporting Gazebo Model Path
In px4_simulation_stack directory,
```bash
echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:$(pwd)/Tools/sitl_gazebo/models" >> ~/.bashrc
```
If you write these settings in .bashrc and .profile,  
do not forget to source them for the first time.  

## Run Gazebo Remotely

### Export Environment Variables
In server machine, run
```bash
export ROS_MASTER_URI=http://server_ip:11311
export ROS_IP=host_ip_server
```
In client machine, run
```bash
export ROS_MASTER_URI=http://server_ip:11311
export ROS_IP=host_ip_client
export GAZEBO_MASTER_URI=http://server_ip:11345
```

You can check host_ip of each machine by running
```bash
hostname -I
```

### Run Launch Files
#### Gazebo
If you want to run only Gazebo, run the following command in server machine.
```bash
roslaunch px4_simulation_stack gzserver.launch
```
In client machine, run
```bash
roslaunch px4_simulation_stack gzclient.launch
```

#### Gazebo and PX4 SITL
If you want to run Gazebo and PX4 SITL, run the following command in server machine.
```bash
roslaunch px4_simulation_stack posix_sitl_remote_server.launch
```
In client machine, run
```bash
roslaunch px4_simulation_stack gzclient.launch
```

## Notes about Launch Files

### hexa_j153.launch
In order to launch `hexa_j153.launch`, you need to put gazebo models of gazebo model database in your `GAZEBO_MODEL_PATH`.
Gazebo models of gazebo model database can be downloaded from https://bitbucket.org/osrf/gazebo_models/overview.

This launch file needs Gazebo 7.4.0 or newer to launch because of [this](http://answers.gazebosim.org/question/18014/gazebo-7-ambulance-model-and-other-invalid-mesh-filename-extension-crash/) problem.
Installation of Gazebo from OSRF repository is described [here](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install).
Note that ROS Kinetic should be used with Gazebo 7.x series according to [this page](http://gazebosim.org/tutorials?tut=ros_wrapper_versions).
