#ifndef ORIENTATION_H
#define ORIENTATION_H

#include "util.h"

struct OrientationCommands {
	const std::vector<CartesianCoordinate> & CartesianMap, 
	float averageOrientation, 
	float currentOrientation, 
	float wall
};

static const float MAX_DIFF = (3*M_PI) / 4;
