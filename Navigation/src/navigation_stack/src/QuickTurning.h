#ifndef QUICK_TURNING_H
#define QUICK_TURNING_H

#include "util.h"

struct TurningCommands {
  VehicleGear gear;
  Direction   direction; 
};

static const float TURNING_ANGLE_BUFFER = 0.174533;

static const BoxCoordinates FORWARD_LEFT_BOX   = {-0.46, 0.3, -0.3, 0.46};
static const BoxCoordinates FORWARD_RIGHT_BOX  = {-0.3, 0.46, -0.3, 0.46};
static const BoxCoordinates BACKWARD_LEFT_BOX  = {-0.46, 0.3, -0.46, 0.3};
static const BoxCoordinates BACKWARD_RIGHT_BOX = {-0.3, 0.46, -0.46, 0.3};

// Forward Declarations
TurningCommands TurnVehicle( const std::vector<CartesianCoordinate> & CartesianMap,
  float desiredOrientation, float currentOrientation );

#endif
