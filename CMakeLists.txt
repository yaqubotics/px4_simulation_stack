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
  xacro
)

find_package(PCL 1.7 REQUIRED)
find_package(octomap REQUIRED)

find_package(gazebo REQUIRED)
if("${GAZEBO_VERSION}" VERSION_LESS "7.4")
  message(FATAL_ERROR "Gazebo 7.4 or later is required.")
endif()

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

link_libraries(${OCTOMAP_LIBRARIES})

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

# add airframes
include(cmake/add_airframe.cmake)
set(AIRFRAME_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix)
add_airframe(${AIRFRAME_DIR}/10050_iris_lidar_vertical)
add_airframe(${AIRFRAME_DIR}/60010_F550_rd)
add_airframe(${AIRFRAME_DIR}/60030_F550)

# build urdf
include(cmake/build_urdf.cmake)
set(MODEL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Tools/sitl_gazebo/models)
build_urdf(${MODEL_DIR}/F550/xacro/F550.xacro ${MODEL_DIR}/F550/F550.urdf)
build_urdf(${MODEL_DIR}/F550_rd/F550_rd.xacro ${MODEL_DIR}/F550_rd/F550_rd.urdf)
build_urdf(${MODEL_DIR}/F550_wireless/F550_wireless.xacro ${MODEL_DIR}/F550_wireless/F550_wireless.urdf)