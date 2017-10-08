#include "UTurnDetection.h"
#include <iostream>
#include <cmath>
#include <math.h>

bool checkUTurn (float averageOrientation, float currentOrientation,
float wall) {
 
  
	float orientationValue;
	
	//Find difference
	float difference = orientationDiff(currentOrientation, averageOrientation);
  std::cout << "Difference from average orientation: " << difference << std::endl;

	//Return true if the car needs to be reoriented.
	if( abs(difference) > MAX_DIFF) {
		orientationValue = fmod(currentOrientation+M_PI, 2 * M_PI);
		return true;
	}

	return false;
}
