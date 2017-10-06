cmake_minimum_required(VERSION 2.8.3)
project(px4_simulation_stack)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  laser_assembler
  mavros_msgs
  roscpp
  sensor_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_sweep_demo src/simple_sweep_demo.cpp)
add_executable(offb_node src/offb_node.cpp)

target_link_libraries(simple_sweep_demo ${catkin_LIBRARIES})
target_link_libraries(offb_node ${catkin_LIBRARIES})