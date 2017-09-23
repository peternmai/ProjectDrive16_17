#ifndef WALL_DETECTION_H
#define WALL_DETECTION_H

#include "util.h"

// Specify part of the laser scan to look at in meters
static const int SCAN_MIN_X = -5;
static const int SCAN_MIN_Y = -5;

// Specify part of the laser scan to look at in meters [**DO NOT CHANGE**]
static const int SCAN_MAX_X = -SCAN_MIN_X;
static const int SCAN_MAX_Y = -SCAN_MIN_Y;

// Specify how many pixel per meter on opencv mat
static const int SPACING_IN_METER = 20;

// Hough Transform Parameters
static const float RHO             = 1;
static const float THETA           = M_PI / 180;
static const float THRESHOLD       = 1;
static const float MIN_LINE_LENGTH = 15;
static const float MAX_LINE_GAP    = 10;

// Specify angle we want to consider as part of wall going straight (-PI to PI)
static const float LEFT_SLOPE = -1;
static const float RIGHT_SLOPE = 1;
static const float MIN_ANGLE = -M_PI / 2;  
static const float MAX_ANGLE =  M_PI / 2;


// Forward Declaration
float WallDetection( const std::vector<CartesianCoordinate> & CartesianMap,
  float currentImuDirection );

#endif
