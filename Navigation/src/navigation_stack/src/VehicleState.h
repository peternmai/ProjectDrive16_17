#ifndef VEHICLE_STATE_H
#define vehicle_STATE_H

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

static const float MAX_SPEED_FORWARD  =  0.25;          // meters per second
static const float MAX_SPEED_BACKWARD = -0.25;          // meters per second
static const float MAX_VISIBLE_RANGE  =  5;          // meters

static const float MIN_SPEED_FORWARD  =  0.05;
static const float MIN_SPEED_BACKWARD = -0.05;

#endif
