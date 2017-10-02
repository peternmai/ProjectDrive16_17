#ifndef UTIL_H
#define UTIL_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <math.h>
#include <string.h>
#include <vector>

#define PROGRESSBAR_SIZE 50

#define sind(x) (sin(fmod(x, 360)) * M_PI / 180)
#define cosd(x) (cos(fmod(x, 360)) * M_PI / 180)


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

static struct PolarCoordinate CartesianToPolar( float x, float y ) {
  PolarCoordinate coordinate;
  coordinate.distance = sqrt( pow(x, 2) + pow(y, 2) );
  coordinate.radian = fmod( atan( y / x ) -  M_PI/2, 2 * M_PI );
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

float getClosestObstacleInRectangleBeam(
  const std::vector<CartesianCoordinate> & CartesianMap, 
  float beamAngle, float beamWidth, float lidarToFrontDistance, float carWidth);

int totalPointsInPolygon(
  const CartesianCoordinate & tl, const CartesianCoordinate & tr,
  const CartesianCoordinate & bl, const CartesianCoordinate & br,
  const std::vector<CartesianCoordinate> & v);

CartesianCoordinate closestPointInPolygon(
  const CartesianCoordinate & tl, const CartesianCoordinate & tr,
  const CartesianCoordinate & bl, const CartesianCoordinate & br,
  const std::vector<CartesianCoordinate> & v);

void printProgressBar( std::string title, float percentage );

float orientationDiff(float currentOrientation, float desiredOrientation);

bool between(float val, float bound1, float bound2);

#endif
