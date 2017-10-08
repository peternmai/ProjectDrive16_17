#ifndef BEAM_H
#define BEAM_H

#include "util.h"

using std::vector;

static const BoxCoordinates frontLeftBox   = {-0.5, -0.2, 0.2, 0.5};
static const BoxCoordinates frontMiddleBox = {-0.2,  0.2, 0.2, 1.5};
static const BoxCoordinates frontRightBox  = { 0.2,  0.5, 0.2, 0.5};

float beamPath(const float & angleOffset, const float & width, 
  const int & numRects, const vector<CartesianCoordinate> & map);

float obstacleAvoidance(const vector<CartesianCoordinate> & CartesianMap,
  const float & beamAngleOffset, const float & beamWidth, const float & numBeams);

#endif
