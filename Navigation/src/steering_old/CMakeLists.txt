cmake_minimum_required(VERSION 2.8.3)
project(steering)

## Find catkin macros and libraries needed to build package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  msg
)


## Specify config files for package
catkin_package()


## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Make Laser Scan Publisher Node Executable

## Make tf_broadcaster and tf_listener Executable

add_executable(check src/check.c)
target_link_libraries(check ${catkin_LIBRARIES})
