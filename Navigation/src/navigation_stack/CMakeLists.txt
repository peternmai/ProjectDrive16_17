cmake_minimum_required(VERSION 2.8.3)
project(navigation_stack)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  msg
  std_msgs
  sensor_msgs
  ackermann_msgs
  setup
  move_base
  cv_bridge
  image_transport
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########
add_compile_options(-std=c++11)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Make Main Navigation Module Executable
add_executable(drive src/drive.cpp src/util.cpp src/CruiseControl.cpp
src/QuickTurning.cpp src/WallDetection.cpp src/beam.cpp)
add_dependencies(drive ${catkin_EXPORTED_TARGETS})
target_link_libraries(drive ${catkin_LIBRARIES} ${GSL_LIBRARIES})

add_executable(lra_pub src/LaneKeepingAssistance.cpp)
add_dependencies(lra_pub ${catkin_EXPORTED_TARGETS})
target_link_libraries(lra_pub ${catkin_LIBRARIES})
