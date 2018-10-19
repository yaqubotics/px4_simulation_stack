cmake_minimum_required(VERSION 2.8.3)
project(px4_simulation_stack)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  laser_assembler
  mavros_msgs
  roscpp
  sensor_msgs
  pcl_conversions
)

find_package(PCL 1.7 REQUIRED)
catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})


add_definitions(${PCL_DEFINITIONS})

add_executable(simple_sweep_demo src/simple_sweep_demo.cpp)
add_executable(offb_node src/offb_node.cpp)
add_executable(rc_converter src/rc_converter.cpp)

target_link_libraries(simple_sweep_demo
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(offb_node ${catkin_LIBRARIES})
target_link_libraries(rc_converter ${catkin_LIBRARIES})
