#ifndef SPEEDCONTROLLER_H
#define SPEEDCONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <math.h>

#include "util.h"

// Forward Declarations
CartesianCoordinate ClosestObstacleAhead(
  const sensor_msgs::LaserScan::ConstPtr & scan, float min_x, float max_x);

float GetCruiseSpeed( const sensor_msgs::LaserScan::ConstPtr & scan, 
  float max_speed, float max_distance );


#endif
