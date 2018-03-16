cmake_minimum_required(VERSION 2.8.3)
project(ft232h_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
)

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/libftdi1
)

add_executable(
  ft232h_interface src/ft232h_interface.cpp
)
target_link_libraries(
  ft232h_interface ${catkin_LIBRARIES} ftdi1
)
add_dependencies(
  ft232h_interface
  ${catkin_EXPORTED_TARGETS}
)

add_executable(
  ftdi_test_node src/ftdi_test_node.cpp
)
target_link_libraries(
  ftdi_test_node ${catkin_LIBRARIES} ftdi1
)
add_dependencies(
  ftdi_test_node
  ${catkin_EXPORTED_TARGETS}
)
