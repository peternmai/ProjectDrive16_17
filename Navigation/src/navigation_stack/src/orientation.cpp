#include "orientation.h"
#include <iostream>
#include <cmath>
#include <math.h>

bool OrientationVehicle ( float averageOrientation, float currentOrientation, float wall) {

	float orientationValue;
	float simplifiedAvg;
	float simplifiedCurrent;
	std::vector<CartesianCoordinate> map;
	float avg = (averageOrientation + wall) / 2;
	//Chang values to positive
	while(averageOrientation < 0) {
		averageOrientation = avg + 2 * M_PI;
	}
	while(currentOrientation < 0) {
		currentOrientation = avg + 2 * M_PI;
	}
	
	while(wall < 0) {
		wall = avg + 2 * M_PI;
	}

	

	//Simplify
	while(avg > M_PI) {
		simplifiedAvg = avg - 2 * M_PI;
	}

	while(currentOrientation > M_PI) {
		simplifiedCurrent = currentOrientation - 2 * M_PI;
	}

	//Find difference
	float difference = orientationDiff(simplifiedCurrent, simplifiedAvg);

	//Return true if the car needs to be reoriented.
	if(difference > MAX_DIFF) {
		orientationValue = fmod(currentOrientation+M_PI, 2 * M_PI);
		return true;
	}

	return false;
}