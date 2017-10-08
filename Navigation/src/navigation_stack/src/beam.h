 /*****************************************************************************

                                                         Author: Jason Ma
                                                         Date:   Sep 30 2017
                                      drive

 File Name:       beam.h
 Description:     Provides function for finding the angle that will allow the
                  vehicle to move furthest forwards.
 *****************************************************************************/

#ifndef BEAM_H
#define BEAM_H

#include "util.h"

using std::vector;

float beamPath(float angleOffset, float width, int numRects, vector<CartesianCoordinate> map);

#endif /* BEAM_H */
