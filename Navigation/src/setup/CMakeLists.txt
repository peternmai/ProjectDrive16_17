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
##add_executable(laser_pub src/laser_pub_range_sub.cpp )
##add_dependencies(laser_pub ${catkin_EXPORTED_TARGETS} )
##target_link_libraries(laser_pub ${catkin_LIBRARIES} ${GSL_LIBRARIES} )

add_executable(laser_scan_publisher src/LaserScanPublisher.cpp)
##add_dependencies(laser_scan_publisher ${catkin_EXPORTED_TAGETS})
target_link_libraries(laser_scan_publisher ${catkin_LIBRARIES} ${GSL_LIBRARIES})

## Make Laser Scan Publisher and Range Listener Executable
##add_executable(laser_pub_range_sub src/laser_pub_range_sub.cpp)
##add_dependencies(laser_pub_range_sub ${catkin_EXPORTED_TARGETS})
##target_link_libraries(laser_pub_range_sub ${catkin_LIBRARIES} ${GSL_LIBRARIES})

## Make tf_broadcaster and tf_listener Executable
##add_executable(tf_pub src/tf_pub.cpp)
##add_executable(tf_sub src/tf_sub.cpp)
##target_link_libraries(tf_pub ${catkin_LIBRARIES})
##target_link_libraries(tf_sub ${catkin_LIBRARIES})
