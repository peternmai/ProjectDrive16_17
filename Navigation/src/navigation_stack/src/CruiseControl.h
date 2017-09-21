#ifndef CRUISE_CONTROL_H
#define CRUISE_CONTROL_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <math.h>

#include "VehicleState.h"
#include "util.h"

static const float CAR_BUFFER     =  0.5;

static const float FORWARD_MIN_X  = -0.3048;
static const float FORWARD_MAX_X  =  0.3048;

static const float BACKWARD_MIN_X = -0.3048;
static const float BACKWARD_MAX_X =  0.3408;

struct CruiseControl {
  CartesianCoordinate closestCoordinate;
  float               proposed_speed;
  float               proposed_steering_angle;
};

// Forward Declarations
CartesianCoordinate ClosestObstacleAhead(
  const sensor_msgs::LaserScan::ConstPtr & scan, float min_x, float max_x);

CartesianCoordinate ClosestObstacleBehind(
  const sensor_msgs::LaserScan::ConstPtr & scan, float min_x, float max_X);

CruiseControl GetCruiseControl(
  const sensor_msgs::LaserScan::ConstPtr & scan, VehicleGear gear );

#endif
