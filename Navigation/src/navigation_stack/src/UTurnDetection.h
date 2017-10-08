#ifndef U_TURN_DETECTION_H
#define U_TURN_DETECTION_H

#include "util.h"

bool checkUTurn (
	float averageOrientation, 
	float currentOrientation, 
	float wall
);

static const float MAX_DIFF = (2*M_PI) / 4;

#endif
