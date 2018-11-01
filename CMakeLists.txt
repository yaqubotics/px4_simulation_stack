cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0054 NEW)

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

# Airframe file paths
if($ENV{ROS_HOME})
  set(ROS_ROMFS_DIR "$ENV{ROS_HOME}/etc")
else()
  set(ROS_ROMFS_DIR "$ENV{HOME}/.ros/etc")
endif()

set(PACK_ROMFS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ROMFS/px4fmu_common)

if(NOT EXISTS ${ROS_ROMFS_DIR}/init.d-posix/10050_iris_lidar_vertical)
  add_custom_target(iris_lidar_vertical
                    ALL ln -s ${PACK_ROMFS_DIR}/init.d-posix/10050_iris_lidar_vertical ${ROS_ROMFS_DIR}/init.d-posix/)
endif()

if(NOT EXISTS ${ROS_ROMFS_DIR}/init.d-posix/60010_F550_rd)
  add_custom_target(f550_rd
                    ALL ln -s ${PACK_ROMFS_DIR}/init.d-posix/60010_F550_rd ${ROS_ROMFS_DIR}/init.d-posix/)
endif()

if(NOT EXISTS ${ROS_ROMFS_DIR}/init.d-posix/60030_F550)
  add_custom_target(f550
                    ALL ln -s ${PACK_ROMFS_DIR}/init.d-posix/60030_F550 ${ROS_ROMFS_DIR}/init.d-posix/)
endif()

# Doxygen document generation
find_package(Doxygen)
if(DOXYGEN_FOUND)
add_custom_target(gen_doc ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        VERBATIM )
endif()