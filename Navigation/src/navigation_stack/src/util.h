#ifndef UTIL_H
#define UTIL_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <math.h>


struct CartesianCoordinate {
  float x;         // Positive X = to right of center. (meter)
  float y;         // Positive Y = in front of center. (meter)
};

struct PolarCoordinate {
  float distance;  // Distance from center
  float radian;    // 0 Radian = In front of car. PI = Left of Car
};

// Given the polar coordinate specified in the struct above, return the cartesian coordinate
static struct CartesianCoordinate PolarToCartesian( float distance, float radian ) {
  float x = distance * cos( radian + M_PI/2 );
  float y = distance * sin( radian + M_PI/2 );

  CartesianCoordinate coordinate;
  coordinate.x = x;
  coordinate.y = y;
  return coordinate;
}


// Forward Declarations
void FilterLaserScanAngles(const sensor_msgs::LaserScan::ConstPtr & scan,
  float * filtered_scan, float min_radian, float max_radian);

#endif
