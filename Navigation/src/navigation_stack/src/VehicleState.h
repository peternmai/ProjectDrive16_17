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

#define MAX_SPEED_FORWARD  2           // meters per second
#define MAX_SPEED_BACKWARD 2           // meters per second
#define MAX_VISIBLE_RANGE  5           // meters

#endif
