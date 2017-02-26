cmake_minimum_required(VERSION 2.8.3)
project(setup)

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
add_executable(laser_scan_pub src/LaserScanPublisher.cpp )
add_dependencies(laser_scan_pub ${catkin_EXPORTED_TARGETS} )
target_link_libraries(laser_scan_pub ${catkin_LIBRARIES} ${GSL_LIBRARIES} )

## Make Test Node Executable
add_executable( test_node src/Test.cpp )
add_dependencies( test_node ${catkin_EXPORTED_TARGETS} )
target_link_libraries( test_node ${catkin_LIBRARIES} ${GSL_LIBRARIES} )

## Make tf_broadcaster and tf_listener Executable
add_executable(tf_pub src/tf_pub.cpp)
add_executable(tf_sub src/tf_sub.cpp)
target_link_libraries(tf_pub ${catkin_LIBRARIES})
target_link_libraries(tf_sub ${catkin_LIBRARIES})

## Make Laser Scan Publisher Node Executable - Test Version 2
add_executable(laser_scan_pub_v2 src/LaserScanPublisherV2.cpp )
add_dependencies(laser_scan_pub_v2 ${catkin_EXPORTED_TARGETS} )
target_link_libraries(laser_scan_pub_v2 ${catkin_LIBRARIES} ${GSL_LIBRARIES} )

## Make Laser Scan Publisher Node Executable - Test Version 3
add_executable(laser_scan_pub_v3 src/LaserScanPublisherV3.cpp )
add_dependencies(laser_scan_pub_v3 ${catkin_EXPORTED_TARGETS} )
target_link_libraries(laser_scan_pub_v3 ${catkin_LIBRARIES} ${GSL_LIBRARIES} )
