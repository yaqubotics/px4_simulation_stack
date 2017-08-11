# px4_simulation_stack
Package for PX4 SITL Gazebo simulation.  

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
echo "source $(pwd)/Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$(pwd)" >> ~/.profile
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo" >> ~/.profile
```
  
### Exporting Gazebo Model Path
In px4_simulation_stack directory,
```bash
echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:$(pwd)/Tools/sitl_gazebo/models" >> ~/.profile
```
If you write these settings in .bashrc and .profile,  
do not forget to source them for the first time.  
