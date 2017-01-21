cmake_minimum_required(VERSION 2.8.3)
project(setup)

## Find catkin macros and libraries needed to build package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
)


## Specify config files for package
catkin_package()


## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Make Laser Scan Publisher Node Executable
add_executable( publish_laser_scan src/LaserScanPublisher.cpp )
add_dependencies( publish_laser_scan ${catkin_EXPORTED_TARGETS} )
target_link_libraries( publish_laser_scan ${catkin_LIBRARIES} ${GSL_LIBRARIES} )


## Make Transform Publisher Node Executable
add_executable( broadcast_transform src/TransformPublisher.cpp )
add_dependencies( broadcast_transform ${catkin_EXPORTED_TARGETS} )
target_link_libraries( broadcast_transform ${catkin_LIBRARIES} ${GSL_LIBRARIES} )



## Make Test Node Executable
add_executable( test_node src/test.cpp )
add_dependencies( test_node ${catkin_EXPORTED_TARGETS} )
target_link_libraries( test_node ${catkin_LIBRARIES} ${GSL_LIBRARIES} )
