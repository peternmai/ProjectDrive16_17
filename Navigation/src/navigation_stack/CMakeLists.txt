cmake_minimum_required(VERSION 2.8.3)
project(navigation_stack)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  setup
  move_base
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)
