#ifndef VEHICLE_STATE_H
#define vehicle_STATE_H

#include <math.h>

enum VehicleGear {
  forward,
  parked,
  backward
};

enum Direction {
  front,
  back,
  left,
  right
};

static const float MAX_CRUISE_SPEED       =  1;

static const float MAX_OBSTACLE_AVOIDANCE_SPEED_FORWARD  =  0.5;
static const float MAX_OBSTACLE_AVOIDANCE_SPEED_BACKWARD = -0.5;

static const float MAX_VISIBLE_RANGE  =  5;

static const float MIN_SPEED_FORWARD  =  0.15;
static const float MIN_SPEED_BACKWARD = -0.15;

static const float MAX_STEERING_LEFT  =  M_PI / 2;
static const float MAX_STEERING_RIGHT = -M_PI / 2;

#endif
