#ifndef UTIL_H
#define UTIL_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <math.h>
#include <string.h>
#include <vector>

#define PROGRESSBAR_SIZE 50


struct CartesianCoordinate {
  float x;         // Positive X = to right of center. (meter)
  float y;         // Positive Y = in front of center. (meter)
};

struct PolarCoordinate {
  float distance;  // Distance from center
  float radian;    // 0 Radian = In front of car. PI = Left of Car
};

struct BoxCoordinates {
  float min_x;
  float max_x;
  float min_y;
  float max_y;
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

std::vector<CartesianCoordinate> LaserScanToCartesianMap(
  const sensor_msgs::LaserScan::ConstPtr & scan );

std::vector<CartesianCoordinate> CartesianMapBoxFilter(
  const std::vector<CartesianCoordinate> & CartesianMap,
  BoxCoordinates boxCoordinates );

void printProgressBar( std::string title, float percentage );

#endif
